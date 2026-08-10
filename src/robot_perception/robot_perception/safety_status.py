def safety_status_fault_reason(status):
    if not isinstance(status, dict) or status.get('valid') is not True:
        return 'safety_invalid'
    emergency_stop = status.get('emergency_stop')
    if emergency_stop is True:
        return 'emergency_stop'
    if emergency_stop is not False:
        return 'safety_invalid'
    return None
