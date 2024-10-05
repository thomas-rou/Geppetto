def set_mission_status(mission_status):
    with open('/tmp/.mission_status', 'w') as f:
        f.write(mission_status)
def get_mission_status():
    with open('/tmp/.mission_status', 'r') as f:
        return f.read()