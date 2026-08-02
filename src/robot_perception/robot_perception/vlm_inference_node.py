import json
import re
import time

import rclpy
import torch
from cv_bridge import CvBridge
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String
from transformers import BlipForConditionalGeneration, BlipProcessor


class VlmInferenceNode(Node):
    def __init__(self):
        super().__init__('vlm_inference_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('result_topic', '/vlm/perception_result')
        self.declare_parameter('inference_interval', 2.0)
        self.declare_parameter(
            'model_id',
            'Salesforce/blip-image-captioning-base',
        )
        self.declare_parameter('max_new_tokens', 30)
        self.declare_parameter('use_fp16', True)
        self.declare_parameter('target_object', 'bottle')

        image_topic = self.get_parameter('image_topic').value
        result_topic = self.get_parameter('result_topic').value
        inference_interval = self.get_parameter('inference_interval').value
        model_id = self.get_parameter('model_id').value
        self.max_new_tokens = self.get_parameter('max_new_tokens').value
        use_fp16 = self.get_parameter('use_fp16').value
        self.target_object = (
            self.get_parameter('target_object').value.strip().lower()
        )
        if not image_topic:
            raise ValueError('image_topic must not be empty')
        if not result_topic:
            raise ValueError('result_topic must not be empty')
        if inference_interval <= 0.0:
            raise ValueError('inference_interval must be greater than zero')
        if not model_id:
            raise ValueError('model_id must not be empty')
        if self.max_new_tokens <= 0:
            raise ValueError('max_new_tokens must be greater than zero')
        if not self.target_object:
            raise ValueError('target_object must not be empty')

        self.bridge = CvBridge()
        self.latest_image = None
        self.frame_reported = False
        self.waiting_reported = False
        self.device = torch.device(
            'cuda' if torch.cuda.is_available() else 'cpu'
        )
        self.model_dtype = (
            torch.float16
            if self.device.type == 'cuda' and use_fp16
            else torch.float32
        )

        if use_fp16 and self.device.type != 'cuda':
            self.get_logger().warning(
                'CUDA is unavailable; falling back to CPU with FP32'
            )

        self.load_model(model_id)

        self.result_publisher = self.create_publisher(
            String,
            result_topic,
            10,
        )

        self.create_subscription(
            RosImage,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(inference_interval, self.process_latest_image)

        self.get_logger().info(
            f'Waiting for images on {image_topic}; '
            f'processing interval={inference_interval:.1f}s'
        )

    def load_model(self, model_id):
        self.get_logger().info(
            f'Loading {model_id} from local cache on {self.device.type}'
        )
        started = time.perf_counter()

        try:
            self.processor = BlipProcessor.from_pretrained(
                model_id,
                use_fast=False,
                local_files_only=True,
            )
            self.model = BlipForConditionalGeneration.from_pretrained(
                model_id,
                dtype=self.model_dtype,
                local_files_only=True,
            ).to(self.device)
            self.model.eval()
            if self.device.type == 'cuda':
                torch.cuda.synchronize()
        except Exception as error:
            self.get_logger().fatal(f'BLIP model loading failed: {error}')
            raise

        load_ms = (time.perf_counter() - started) * 1000
        self.get_logger().info(
            f'BLIP model ready on {self.device.type}; load={load_ms:.0f}ms'
        )

    def image_callback(self, message):
        self.latest_image = message

    def process_latest_image(self):
        if self.latest_image is None:
            if not self.waiting_reported:
                self.get_logger().warning('No camera image received yet')
                self.waiting_reported = True
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(
                self.latest_image,
                desired_encoding='rgb8',
            )
        except Exception as error:
            self.get_logger().error(f'Image conversion failed: {error}')
            self.publish_invalid_result()
            return

        if not self.frame_reported:
            height, width = frame.shape[:2]
            self.get_logger().info(f'Camera frame ready: {width}x{height}')
            self.frame_reported = True

        try:
            image = PilImage.fromarray(frame)
            inputs = self.processor(images=image, return_tensors='pt')
            inputs = {
                name: tensor.to(
                    device=self.device,
                    dtype=(
                        self.model_dtype
                        if tensor.is_floating_point()
                        else tensor.dtype
                    ),
                )
                for name, tensor in inputs.items()
            }

            if self.device.type == 'cuda':
                torch.cuda.synchronize()
            started = time.perf_counter()
            with torch.inference_mode():
                output_ids = self.model.generate(
                    **inputs,
                    max_new_tokens=self.max_new_tokens,
                )
            if self.device.type == 'cuda':
                torch.cuda.synchronize()
            latency_ms = (time.perf_counter() - started) * 1000

            caption = self.processor.decode(
                output_ids[0],
                skip_special_tokens=True,
            )
        except Exception as error:
            self.get_logger().error(f'BLIP inference failed: {error}')
            self.publish_invalid_result()
            return

        result = self.parse_caption(caption, latency_ms)
        self.publish_result(result)
        self.get_logger().info(
            f'[VLM] object={result["object"]} '
            f'position={result["position"]} '
            f'action={result["suggested_action"]} '
            f'valid={str(result["valid"]).lower()} '
            f'latency={result["latency_ms"]}ms'
        )

    def parse_caption(self, caption, latency_ms):
        normalized_caption = caption.lower()
        target_pattern = rf'\b{re.escape(self.target_object)}\b'
        target_detected = re.search(
            target_pattern,
            normalized_caption,
        ) is not None

        words = set(re.findall(r'[a-z]+', normalized_caption))
        positions = []
        if 'left' in words:
            positions.append('left')
        if 'right' in words:
            positions.append('right')
        if 'center' in words or 'middle' in words:
            positions.append('center')

        valid = target_detected and len(positions) == 1
        position = positions[0] if valid else 'unknown'
        action_map = {
            'left': 'turn_right',
            'right': 'turn_left',
            'center': 'stop',
        }

        return {
            'object': self.target_object if target_detected else 'unknown',
            'position': position,
            'risk': 'unknown',
            'suggested_action': action_map.get(position, 'stop'),
            'caption': caption,
            'latency_ms': int(round(latency_ms)),
            'valid': valid,
        }

    def publish_invalid_result(self):
        self.publish_result({
            'object': 'unknown',
            'position': 'unknown',
            'risk': 'unknown',
            'suggested_action': 'stop',
            'caption': '',
            'latency_ms': 0,
            'valid': False,
        })

    def publish_result(self, result):
        message = String()
        message.data = json.dumps(result, ensure_ascii=True)
        self.result_publisher.publish(message)

    def destroy_node(self):
        if hasattr(self, 'model'):
            del self.model
        if self.device.type == 'cuda':
            torch.cuda.empty_cache()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = VlmInferenceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except KeyboardInterrupt:
                    pass


if __name__ == '__main__':
    main()
