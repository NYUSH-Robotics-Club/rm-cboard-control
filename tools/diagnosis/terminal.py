"""Raw bidirectional RTT terminal. Example: hello 1 / status 2 / read 3 1 1 7."""
import argparse
import socket
import sys
import threading


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=9090)
    args=parser.parse_args()
    with socket.create_connection(('127.0.0.1',args.port),timeout=3) as connection:
        connection.settimeout(None)
        def receive():
            try:
                while data:=connection.recv(8192):
                    sys.stdout.buffer.write(data)
                    sys.stdout.buffer.flush()
            except OSError:
                pass
        threading.Thread(target=receive,daemon=True).start()
        print('RTT channel 0 connected. Use unique nonzero tokens. Ctrl-C exits; no motor command is sent automatically.',flush=True)
        try:
            for line in sys.stdin:
                connection.sendall(line.encode('ascii'))
        except KeyboardInterrupt:
            pass


if __name__=='__main__':
    main()
