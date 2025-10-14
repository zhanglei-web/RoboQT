import zmq  
import pyarrow as pa  
from dora import Node  
  
ipc_address = "ipc:///tmp/dora-robot"  

context = zmq.Context()  
socket = context.socket(zmq.PAIR)  
socket.bind(ipc_address)
print(f"Socket bound to {ipc_address}", flush=True)  
socket.setsockopt(zmq.SNDHWM, 2000)  
socket.setsockopt(zmq.SNDBUF, 2**25)  
running_server = True

def main():  
    node = Node()  
    for event in node: 
        if event["type"] == "INPUT":  
            event_id = event["id"]  
            buffer_bytes = event["value"].to_numpy().tobytes()
            try:  
                socket.send_multipart([
                    event_id.encode('utf-8'),
                    buffer_bytes
                ], flags=zmq.NOBLOCK)
            except zmq.Again:
                pass  
                print("[WARN] 发送缓冲区满，丢弃数据", flush=True)
            except Exception as e:  
                print(f"[ERROR] Send failed: {e}", flush=True)
                  
        elif event["type"] == "STOP":  
            break  
    
    running_server=False
    socket.close()  
    context.term()  
  
if __name__ == "__main__":  
    main()
