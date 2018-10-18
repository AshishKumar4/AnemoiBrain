from socket import * 

IP = "127.0.0.1"
PORT = 8194

ss = socket(AF_INET, SOCK_STREAM)
ss.connect((IP, PORT))
