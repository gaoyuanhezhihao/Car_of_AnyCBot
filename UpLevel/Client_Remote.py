# Client_Indoor.py
import socket

SERVERIP = "127.0.0.1"
SERVERPORT = 8888
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
while True:
    try:
        s.connect((SERVERIP, SERVERPORT))
        s.sendall("Boss\nBoss1\nConnectCarCar\nCar1")
        while True:
            msg = raw_input("Your command:")
            if msg in ['l', 'r']:
                angle = raw_input("angle:")
                msg +='\n'
                msg += angle
            s.sendall(msg+'\n')
            print "recv:", s.recv(1024)
    except Exception, e:
        print "*** connection failed. \n", e, "\nRetrying"
