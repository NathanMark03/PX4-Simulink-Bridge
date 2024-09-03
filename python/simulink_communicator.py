import socket
import time

# def format_byte(x):
#     return "b\'"+ x +"\'"

def _udp_decode(x):
    j = 0
    data = []
    for elem in range(0, len(x)):
        if(j % 2==0):
            data.append(x[elem])
        j += 1
    return data

def connect_socket(ip, port):
    sock = socket.socket(
        socket.AF_INET, #Internet
        socket.SOCK_DGRAM #UDP
    )
    try:
        sock.bind((ip, port))
    except:
        print("model failed to connect, please check network connection \
            and that the selected port is not already in use. \
            Also ensure that the IP and Port you are broadcasting from \
            is correct"
            )

    return sock

def plant_data(sock, buffsize = 1024):
    data = sock.recv(buffsize)

    return _udp_decode(data)

def send_data(sock, data):
    sock.send(data)


if __name__ == "__main__":

    SEND = 0

    UDP_IP = "172.24.144.1" # 127.0.0.1 172.24.147.121
    UDP_IN_PORT = 5005
    UDP_OUT_PORT = 5006

    in_sock = socket.socket(
        socket.AF_INET, # Internet
        socket.SOCK_DGRAM) # UDP
    
    out_sock = socket.socket(
        socket.AF_INET, # Internet
        socket.SOCK_DGRAM) # UDP

    in_sock.bind((UDP_IP, UDP_IN_PORT))

    out_sock.connect((UDP_IP, UDP_OUT_PORT))

    if SEND:
        data = [1, 2, 3, 6]

        bytes_data = bytes(data)

        send_data(out_sock, bytes_data)
    
    else:
        data = in_sock.recv(1024)

        print("raw data: ", data)

        print(_udp_decode(data))