import socket

def get_local_ip():
    # Create a dummy socket to get the local IP
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            # Doesn't have to be reachable, just used to determine the local IP
            s.connect(('8.8.8.8', 80))
            return s.getsockname()[0]
    except Exception as e:
        return f"Unable to get local IP: {e}"
    
def write_environment_file(ip_address):
    file_path = '../application/client/src/environments/environment.ts'    
    with open(file_path, 'w') as f:
        f.write(f"export const environment = {{\n")
        f.write(f"    production: false,\n")
        f.write(f"    serverUrl: 'http://{ip_address}:3000/api',\n")
        f.write(f"    serverUrlRoot: 'http://{ip_address}:3000/',\n")
        f.write(f"}};\n")

if __name__ == "__main__":
    ip_address = get_local_ip()
    write_environment_file(ip_address)