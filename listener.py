import zmq
import threading
import datetime
import metaweardata_pb2

# Fields in protobuf packets:
# For battery packets:
#   timestamp
#   w = battery percent
#   x = battery millivolts
#
# For switch packets:
#   timestamp
#   w = 1.0 if switch pressed, 0.0 if switch releaased
#
# For fused packets:
#   timestamp
#   acc.{x,y,z}
#   gyro.{x,y,z}
#   extra = packet counter

def deserialize(msg):
    packet = metaweardata_pb2.MetaWearData()
    packet.ParseFromString(msg)

    data = {
        "timestamp_sensor": packet.timestamp / 1000,
        "timestamp": datetime.datetime.now().timestamp()}  # return timestamp python-style in seconds
    if packet.type == metaweardata_pb2.MetaWearData.SWITCH:
        data["type"] = "switch"
        data["switch_pressed"] = True if packet.w else False

    elif packet.type == metaweardata_pb2.MetaWearData.BATTERY:
        data["type"] = "battery"
        data["battery_percent"] = packet.w
        data["battery_millivolts"] = packet.x

    elif packet.type == metaweardata_pb2.MetaWearData.FUSED:
        data["type"] = "fused"
        data["acc_x"] = packet.acc.x
        data["acc_y"] = packet.acc.y
        data["acc_z"] = packet.acc.z
        data["gyro_x"] = packet.gyro.x
        data["gyro_y"] = packet.gyro.y
        data["gyro_z"] = packet.gyro.z
        data["counter"] = packet.extra
    elif packet.type in (metaweardata_pb2.MetaWearData.ACC, metaweardata_pb2.MetaWearData.GYRO):
        print("Warning: Receiving packets of type {}. Is your sensor running the latest firmware?".format(packet.type))
        return {}
    else:
        print("Warning: Packet type {} not implemented".format(packet.type))
        return {}
        
    return data

class Listener(object):
    """
    This class can be used to listen to data streamed over a PUB socket

    After the creation of the listener, start listening by calling start.

    You can register callbacks that will receive new data by using
    register_callback.
    Note that all callbacks should accept a single argument, as this will be new
    received data.
    """

    def __init__(self, address, bind=False):
        """
        Create the Listener by specifying the address to connect to

        :param address: Address of the PUB socket (string, proto://host:port)
        :param bind: Boolean indicating if the Listener should bind or connect
            to the PUB server, default is True (bind).
        """
        self.all_received = []
            
        self.ioloop = None

        ctx = zmq.Context()
        self.listener = ctx.socket(zmq.SUB)
        self.listener.setsockopt_string(zmq.SUBSCRIBE, "")

        if bind:
            self.listener.bind(address)
        else:
            self.listener.connect(address)
        self.poller = zmq.Poller()
        self.poller.register(self.listener, zmq.POLLIN)

        self.callbacks = []
        self.active = True
        self.listener_thread = None

    def _stream_callback(self, msg):
        #msg is multipart, extract it first
        self.call_callbacks(msg[0])

    def start(self, verbose=False):
        if self.ioloop is None:
            self.listener_thread = threading.Thread(target=self._run,
                                                    args=(verbose,))
            self.listener_thread.start()

    def register_callback(self, callback, type_filter = None):
        """
        Register a new callback to be called when new data is available.
        When closed() is called, all callbacks will be called with None as argument.

        :param callback: Callable, has to accept a single argument (received data)
        :param type_filter: optional, list/tuple of packet types ("switch",
            "battery", "fused"). If specified, only packets of these types will be
            received. If None, all packets will be received.
        """
        type_filter = type_filter if type_filter is not None else ("switch","battery","fused")
        self.callbacks.append((callback, type_filter))
    
    def get_recorded(self, filter_types=None, as_type=None):
        """
        Get all received data. With default parameters, this return a list of
        packets (dicts). The packets can be filtered so only packets of matching
        types will be returned. Optionally a different format can be selected.
        
        :param filter_types: Only return packets of these types
        :param datatype: If None, return a list of dicts. If "dataframe", return a
            pandas DataFrame (will contain columns with missing values, if more than
            one filter_type is specified).
        """
        data = self.all_received[:]
        if filter_types:
            data = [d for d in data if d["type"] in filter_types]
        
        if as_type and as_type.lower() == "dataframe":
            import pandas as pd
            data = pd.DataFrame(data)
        return data

    def call_callbacks(self, msg):
        """
        Call all registered callback functions with given messages as argument.

        :param msg: The message to pass to callbacks
        """
        deserialized = deserialize(msg)
        if deserialized:
            self.all_received.append(deserialized)
            for callback, type_filter in self.callbacks:
                if deserialized["type"] in type_filter:
                    callback(deserialized)

    def _run(self, verbose=False):
        """
        Listen and receive new data.

        :param verbose: If true: print all received data objects
        """
        if self.ioloop is None:
            while self.active:
                evts = dict(self.poller.poll(600))
                if self.listener in evts:
                    data = self.listener.recv()
                    if verbose:
                        print(data)
                    self.call_callbacks(data)

    def close(self):
        """
        Close the background listening thread and the used sockets.

        """
        self.active = False
        if self.ioloop is None:
            if self.listener_thread is not None:
                self.listener_thread.join()
                self.listener.close()
                # for callback in self.callbacks:
                #    callback(None)
        else:
            self.stream.close()
