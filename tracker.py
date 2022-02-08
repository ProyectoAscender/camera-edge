from datetime import datetime
# from pycompss.api.parameter import *
# from pycompss.api.task import task
# from pycompss.api.api import compss_barrier, compss_wait_on
# from pycompss.api.constraint import constraint
from socket import timeout
from utils import pixel2GPS
#import deduplicator as dd
#import paho.mqtt.client as mqtt
import track
import socket
import os

NUM_ITERS = 400
# NUM_ITERS_POLLUTION = 25
SNAP_PER_FEDERATION = 15
N = 5
NUM_ITERS_FOR_CLEANING = 300
CD_PROC = 0

# pollution_file_name = "pollution.csv"


# @task(returns=3, list_boxes=IN, trackers=IN, cur_index=IN, init_point=IN)
def execute_tracking(list_boxes, trackers, cur_index, init_point):
    return track.track2(list_boxes, trackers, cur_index, init_point)

    def stand_dect(self, pred, im0_shape):
        """
            Processing detection for one frame 
            Args:
                pred (list): Detections in original input image coordinates
                    [num_dect, 6] : [x_tl, y_tl, x_br, y_br, score[0:1], classe = coco_label-1]
                    tl: top_left, br: bottom_right
                im0_shape (narray): Original input shape [h, w, c]
            Returns:
                    pred (list): Detections normalized between 0 to 1  
                    [num_dect, 6] : [x_tl, y_tl, x_br, y_br, score[0:1], classe = coco_label]
                    tl: top_left, br: bottom_right

        """

        ## Reescale detection to be normalized between 0 to 1
        pred = self.norm_dect(pred, im0_shape) 
        
        ## Classe type in coco format 80 classes [1:81]
        pred2 = []
        for i, det in enumerate(pred):  # detections per image
            if det is not None and len(det):
                det[:, 5] = det[:, 5] + 1
                ## convert to numpy 
                pred2.append(det.cpu().detach().numpy())

        return pred2


# @task(returns=7,)
def receive_boxes(socket_ip, dummy):
    import struct
    import time
    import traceback

    socket_port = 5559
    if ":" in socket_ip:
        socket_ip, socket_port = socket_ip.split(":")
        socket_port = int(socket_port)

    message = b""
    cam_id = None   
    timestamp = None
    boxes = None    
    box_coords = None
    init_point = None
    no_read = True
    frame_number = -1

    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    serverSocket.sendto(b"A", (socket_ip, socket_port))

    double_size = unsigned_long_size = 8
    int_size = float_size = 4
    boxes = []

    while no_read:
        try:
            no_read = False
            message, address = serverSocket.recvfrom(16000)

            flag = len(message) > 0
            # This flag serves to know if the video has ended
            cam_id = struct.unpack_from("i", message[1:1 + int_size])[0]
            timestamp = struct.unpack_from("Q", message[1 + int_size:1 + int_size + unsigned_long_size])[0]
            box_coords = []
            lat, lon = struct.unpack_from("dd", message[1 + int_size + unsigned_long_size:1 + int_size + unsigned_long_size
                                                                                        + double_size * 2])
    
            init_point = (lat, lon)
            for offset in range(1 + int_size + unsigned_long_size + double_size * 2, len(message),
                                double_size * 10 + int_size + 1 + float_size * 4):
                north, east, frame_number, obj_class = struct.unpack_from('ddIc', message[
                                                                                offset:offset + double_size * 2 + int_size + 1])
                x, y, w, h = struct.unpack_from('ffff', message[offset + double_size * 2 + int_size + 1:offset + double_size * 2
                                                                                + int_size + 1 + float_size * 4])
                boxes.append(track.obj_m(north, east, frame_number, ord(obj_class), int(w), int(h), int(x), int(y), 0.0))
                lat_ur, lon_ur, lat_lr, lon_lr, lat_ll, lon_ll, lat_ul, lon_ul = struct.unpack_from('dddddddd', message[
                                                                                offset + double_size * 2 + int_size + 1 +
                                                                                float_size * 4:])
                box_coords.append((lat_ur, lon_ur, lat_lr, lon_lr, lat_ll, lon_ll, lat_ul, lon_ul))
        except socket.error as e:
            no_read = True
            traceback.print_exc()

    serverSocket.close()
    return cam_id, timestamp, boxes, dummy, box_coords, init_point, frame_number


# @constraint(AppSoftware="xavier")
# @task(trackers_list=COLLECTION_IN, cam_ids=COLLECTION_IN, foo_dedu=INOUT, frames=COLLECTION_IN)
def deduplicate(trackers_list, cam_ids, foo_dedu, frames):
    return_message = dd.compute_deduplicator(trackers_list, cam_ids, frames)
    return return_message, foo_dedu

#dump2(cam_ids[index], timestamps[index],frames[index], list_boxes, info_for_deduplicator[index], box_coords[index])
def dump2(id_cam, ts, frame, list_boxes, info_for_deduplicator, box_coords, fps):
    data = {}
    data['ts'] = frame/fps
    json_data = json.dumps(data)


def dump(id_cam, ts, trackers, iteration, list_boxes, info_for_deduplicator, box_coords):
    import pygeohash as pgh
    import os
    filename = "singlecamera.in"
    if not os.path.exists(filename):
        f = open(filename, "w+")
        f.close()
    with open(filename, "a+") as f:
        # for i, tracker in enumerate([t for t in trackers if t.traj[-1].frame == iteration]):
        for i, tracker in enumerate(trackers):
            print(f'inside dump -- i1:{i} --- tracker.id:{tracker.id}')
            if tracker.id not in [t.id for t in trackers if t.traj[-1].frame == iteration]:
                continue
            print(f'inside dump -- i2:{i}')
            print(info_for_deduplicator[i])
            lat = info_for_deduplicator[i][0]  # round(info_for_deduplicator[i][0], 14)
            lon = info_for_deduplicator[i][1]  # round(info_for_deduplicator[i][1], 14)
            geohash = pgh.encode(lat, lon, precision=7)
            cl = info_for_deduplicator[i][2]
            speed = abs(tracker.ekf.xEst.vel)  # info_for_deduplicator[i][3]
            yaw = tracker.ekf.xEst.yaw  # info_for_deduplicator[i][4]
            pixel_x = info_for_deduplicator[i][6]  # OR list_boxes[tracker.idx].x  # pixels[tracker.idx][0]
            pixel_y = info_for_deduplicator[i][7]  # pixels[tracker.idx][1]
            f.write(
                # f"{id_cam} {iteration} {ts} {cl} {lat:.14f} {lon:.14f} {geohash} {speed} {yaw} {id_cam}_{tracker.id} \
                f"{id_cam} {iteration} {ts} {cl} {lat} {lon} {geohash} {speed} {yaw} {id_cam}_{tracker.id} {pixel_x} \
                {pixel_y} {list_boxes[tracker.idx].w} {list_boxes[tracker.idx].h} {box_coords[tracker.idx][0]} \
                {box_coords[tracker.idx][1]} {box_coords[tracker.idx][2]} {box_coords[tracker.idx][3]} \
                {box_coords[tracker.idx][4]} {box_coords[tracker.idx][5]} {box_coords[tracker.idx][6]} \
                {box_coords[tracker.idx][7]}\n")


# @constraint(AppSoftware="xavier")
# @task(returns=1, trackers_list=COLLECTION_IN, count=IN, kb=IN)
def persist_info_accumulated(trackers_list, count, kb):
    from CityNS.classes import EventsSnapshot
    snapshot_alias = "events_" + str(count)
    snapshot = EventsSnapshot(snapshot_alias, print_federated_events=True, trigger_modena_tp=False)
    snapshot.make_persistent()
    for trackers in trackers_list:
        snapshot.add_events_from_trackers(trackers, kb)  # create events inside dataclay
    return snapshot


# @constraint(AppSoftware="xavier")
# @task(returns=1, trackers=IN, count=IN, kb=IN, with_pollution=IN)
def persist_info(trackers, count, kb, with_pollution):
    classes = ["person", "car", "truck", "bus", "motor", "bike", "rider", "traffic light", "traffic sign", "train",
               "class11", "class12", "class13", "class14", "class15", "class16", "class17", "class18","class19",
               "class20", "class21", "class22", "class23", "class24", "class25", "class26", "class27", "class28",
               "class29", "class30", "class31"]
    from CityNS.classes import EventsSnapshot
    snapshot_alias = "events_" + str(count)
    snapshot = EventsSnapshot(snapshot_alias, print_federated_events=True, trigger_modena_tp=False)
    snapshot.make_persistent()
    snapshot.add_events_from_trackers(trackers, kb)  # create events inside dataclay
    if with_pollution:
        if not os.path.exists(pollution_file_name):
            with open(pollution_file_name, "w") as f:
                f.write("VehID, LinkID, Time, Vehicle_type, Av_link_speed\n")
        for index, ev in enumerate(trackers[1]):
            if classes[ev[2]] in ["car", "bus"]:
                obj_type = (classes[ev[2]]).title()
            elif classes[ev[2]] in ["class20", "class30", "class31"]:
                obj_type = "car"
            elif classes[ev[2]] == "truck":
                obj_type = "HDV"
            else:
                continue
            with open(pollution_file_name, "a") as f:
                f.write(f"{ev[0]}_{ev[1]}, {ev[0]}, {trackers[0]}, {obj_type}, {ev[3]}\n")  # TODO: average link speed
    return snapshot


# @constraint(AppSoftware="xavier")
# @task(snapshot=IN, backend_to_federate=IN)
def federate_info(snapshot, backend_to_federate):
    snapshot.federate_to_backend(backend_to_federate)


# @task(snapshots=COLLECTION_IN, backend_to_federate=IN)
def federate_info_accumulated(snapshots, backend_to_federate):
    for snapshot in snapshots:
        snapshot.federate_to_backend(backend_to_federate)


# @task(kb=IN, foo=INOUT)
def remove_objects_from_dataclay(kb, foo):
    kb.remove_old_snapshots_and_objects(int(datetime.now().timestamp() * 1000), True)
    return foo


# @constraint(AppSoftware="xavier")
# @task(foo=INOUT)
def analyze_pollution(foo):
    a = 'oquesea'
    b = pollution_file_name
    c = 'stream.csv'
    os.system(
        f"scp {pollution_file_name} esabate@192.168.7.32:/m/home/esabate/pollutionMap/phemlight-r/in/ && rm {pollution_file_name}")
    os.system(f"ssh esabate@192.168.7.32 nohup bash dockerScripts/phemlightCommand.sh  {a} {b} {c} &>/dev/null & ")
    return foo


# @task(is_replicated=True)
def init_task():
    pass
    # import uuid
    # # from CityNS.classes import DKB, Event, Object, EventsSnapshot
    # # kb = DKB()
    # kb.make_persistent("FAKE_" + str(uuid.uuid4()))
    # # kb.get_objects_from_dkb()
    # snap = EventsSnapshot("FAKE_SNAP_" + str(uuid.uuid4()))
    # snap.make_persistent("FAKE_SNAP_" + str(uuid.uuid4()))
    # snap.snap_alias
    # event = Event()
    # event.make_persistent("FAKE_EVENT_" + str(uuid.uuid4()))
    # obj = Object("FAKE_OBJ_" + str(uuid.uuid4()), "FAKE")
    # obj.make_persistent("FAKE_OBJ_" + str(uuid.uuid4()))
    # obj.get_events_history(20)


# @constraint(AppSoftware="nvidia")
# @task(returns=3, trackers_list=IN, tracker_indexes=IN, cur_index=IN)
def boxes_and_track(socket_ip, trackers_list, tracker_indexes, cur_index):
    _, _, list_boxes, _ = receive_boxes(socket_ip, 0)
    return execute_tracking(list_boxes, trackers_list, tracker_indexes, cur_index)


def execute_trackers(socket_ips,  with_pollution):
    import uuid
    import time
    import sys
    import os
    # from dataclay.api import register_dataclay, get_external_backend_id_by_name

    trackers_list = [[]] * len(socket_ips)
    cur_index = [0] * len(socket_ips)
    info_for_deduplicator = [0] * len(socket_ips)
    snapshots = list()  # accumulate snapshots
    cam_ids = [0] * len(socket_ips)
    timestamps = [0] * len(socket_ips)
    deduplicated_trackers_list = []  # TODO: accumulate trackers
    box_coords = [0] * len(socket_ips)
    frames = [0] * len(socket_ips)

    #federation_ip, federation_port = "192.168.7.32", 11034  # TODO: change port accordingly
    #dataclay_to_federate = register_dataclay(federation_ip, federation_port)
    #external_backend_id = get_external_backend_id_by_name("DS1", dataclay_to_federate)

    i = 0
    reception_dummies = [0] * len(socket_ips)
    start_time = time.time()
    foo_dedu = foo = None
    print('----------------------')
    while i < NUM_ITERS:
        for index, socket_ip in enumerate(socket_ips):
            print(f'i is :{i}')
            print(f'index is :{index}')
            cam_ids[index], timestamps[index], list_boxes, reception_dummies[index], box_coords[index], init_point, frames[index] = \
                receive_boxes(socket_ip, reception_dummies[index])
            print(f'cam_ids[index] is {cam_ids[index]}')
            print(f'timestamps[index] is {timestamps[index]}')             
            print(f'Len of list_boxes is {len(list_boxes)}')
            print(f'list_boxes is {list_boxes}')   
            print(f'reception_dummies[index] is {reception_dummies[index]}') 
            print(f'Len of box_coords[index] is {len(box_coords[index])}') 
            print(f'box_coords[index] is {box_coords[index]}') 
            print(f'init_point is {init_point}') 
            print(f'frames[index] is {frames[index]}')     
            #(41.338343434224, 2.095856231000018, 1, 0, 0, 0, 782, 512, 28, 24)
            #info is a tuple of <lat (float), lon (float), category (int), velocity (uint8_t), yaw (uint8_t), pixel_x (int), pixel_y (int), pixel_y (int), pixel_y (int), frame (int)>
            trackers_list[index], cur_index[index], info_for_deduplicator[index] = execute_tracking(list_boxes,
                                                                                                    trackers_list[index],
                                                                                                    cur_index[index],
                                                                                                    init_point)
            print(f'Len of trackers_list[index] is {len(trackers_list[index])}')
            print(f'trackers_list[index] is {trackers_list[index]}')
            print(f'cur_index[index] is {cur_index[index]}')
            print(f'Len of info_for_deduplicator[index] is {len(info_for_deduplicator[index])}')
            print(f'info_for_deduplicator[index] is {info_for_deduplicator[index]}')

   

            #dump(cam_ids[index], timestamps[index], trackers_list[index], frames[index], list_boxes, info_for_deduplicator[index], box_coords[index])
            dump2(cam_ids[index], timestamps[index], frames[index], list_boxes, info_for_deduplicator[index], box_coords[index])

            #print(info_for_deduplicator)

        # deduplicated_trackers, foo_dedu = deduplicate(info_for_deduplicator, cam_ids, foo_dedu, frames)  # or frames appended inside info_for_dedu in tracking

   
        # snapshot = persist_info(deduplicated_trackers, i, kb, with_pollution)
     
        # federate_info(snapshot, external_backend_id)
        i += 1
        # if i != 0 and i % NUM_ITERS_POLLUTION == 0 and with_pollution:
        #     print("Executing pollution")
        #     foo = analyze_pollution(foo)
        # if i != 0 and i % NUM_ITERS_FOR_CLEANING == 0:
        #     # compss_barrier()
        #     # delete objects based on timestamps
        #     foo = remove_objects_from_dataclay(kb, foo)

    # compss_barrier()
    end_time = time.time()
    print("Exec Inner Time: " + str(end_time - start_time))
    print("Exec Inner Time per Iteration: " + str((end_time - start_time) / NUM_ITERS))


def on_message(client, userdata, message):
    import time
    global CD_PROC
    CD_PROC += 1
    received_time = time.time()
    msg = str(message.payload.decode('utf-8'))
    print(f"Received message = \"{msg}\" at time {received_time}")
    f = open("cd_log.txt", "a")
    f.write(msg)
    f.close()


def publish_mqtt(client):
    client.publish("test", "Start of the execution of the COMPSs workflow")


def register_mqtt():
    client = mqtt.Client()
    try:
        client.connect("192.168.7.42")  # MQTT server in Modena cloud
    except timeout as e:
        print(e)
        print("VPN Connection not active. Needed for MQTT.")
        exit()
    client.on_message=on_message
    client.subscribe("test")
    client.subscribe("tp-out")
    client.subscribe("cd-out")
    return client

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    import argparse	
    import sys
    import time
    import zmq
    # from dataclay.api import init, finish
    # from dataclay.exceptions.exceptions import DataClayException
		
    # Parse arguments to accept variable number of "IPs:Ports"
    parser = argparse.ArgumentParser()
    parser.add_argument("tkdnn_ips", nargs='+')
    parser.add_argument("--mqtt_wait", nargs='?', const=True, type=str2bool, default=False)  # True as default
    parser.add_argument("--with_pollution", nargs='?', const=True, type=str2bool, default=False)  # True as default
    args = parser.parse_args()
    
    # init()
    # from CityNS.classes import DKB

    # Register MQTT client to subscribe to MQTT server in 192.168.7.42
    #if args.mqtt_wait:
    #    client = register_mqtt()
    #    client.loop_start()

    # initialize all computing units in all workers
    num_cus = 8
    for i in range(num_cus):
        init_task()
    # compss_barrier()
    print(f"Init task completed {datetime.now()}")
    input("Press enter to continue...")

    # Publish to the MQTT broker that the execution has started
    if args.mqtt_wait:
        publish_mqtt(client)

    # # Dataclay KB generation
    # try:
    #     kb = DKB.get_by_alias("DKB")
    # except DataClayException:
    #     kb = DKB()
    #     list_objects = ListOfObjects()
    #     list_objects.make_persistent()
    #     kb.list_objects = list_objects
    #     kb.make_persistent("DKB")

    ### ACK TO START WORKFLOW AT tkDNN ###
    for socket_ip in args.tkdnn_ips:
        if ":" not in socket_ip:
            socket_ip += ":5559"
        context = zmq.Context()
        sink = context.socket(zmq.REQ)
        sink.connect(f"tcp://{socket_ip}")
        sink.send_string("")
        sink.close()
        context.term()

    execute_trackers(args.tkdnn_ips, args.with_pollution)

    if args.mqtt_wait:
        while CD_PROC < NUM_ITERS:
            pass

    print("Exiting Application...")
    finish()


if __name__ == "__main__":
    main()

