# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
from sys import platform
import argparse
import numpy as np
from timeit import default_timer as timer
import json
from os.path import join
from tqdm import tqdm
from glob import glob

from easymocap.dataset import CONFIG
from easymocap.dataset import CONFIG
from easymocap.affinity.affinity import ComposedAffinity
from easymocap.affinity.affinity import getDimGroups
from easymocap.affinity.matchSVT import matchSVT
from easymocap.assignment.associate import simple_associate
from easymocap.assignment.group import PeopleGroup

from easymocap.mytools import Timer

from easymocap.socket.base_client import BaseSocketClient

from easymocap.assignment.track import Track2D, Track3D

    #EasymocapExtractVideoFunctions
#region EasymocapExtractVideoFunctions
 
#Get center and scale for bounding box from openpose detections
def bbox_from_openpose(keypoints, rescale=1.2, detection_thresh=0.01):
    """Get center and scale for bounding box from openpose detections."""
    valid = keypoints[:,-1] > detection_thresh
    valid_keypoints = keypoints[valid][:,:-1]
    center = valid_keypoints.mean(axis=0)
    bbox_size = valid_keypoints.max(axis=0) - valid_keypoints.min(axis=0)
    # adjust bounding box tightness
    bbox_size = bbox_size * rescale
    bbox = [
        center[0] - bbox_size[0]/2, 
        center[1] - bbox_size[1]/2,
        center[0] + bbox_size[0]/2, 
        center[1] + bbox_size[1]/2,
        keypoints[valid, 2].mean()
    ]
    return np.array(bbox)

#egy bizonyos json file átalakítása openpose-ból a kívánt formátumra, visszatér dictionary tömbbel
def load_openpose(annotop):
    mapname = {'face_keypoints_2d':'face2d', 'hand_left_keypoints_2d':'handl2d', 'hand_right_keypoints_2d':'handr2d'}
    data = annotop
    out = []
    pid = 0
    for i, d in enumerate(data['people']):
        keypoints = d['pose_keypoints_2d']
        keypoints = np.array(keypoints).reshape(-1, 3)
        annot = {
            'bbox': bbox_from_openpose(keypoints),
            'id': pid + i,
            'keypoints': keypoints,
            'isKeyframe': False
        }
        out.append(annot)
    return out
# convert the 2d pose from openpose
def convert_from_openpose(annotop):
    annots = load_openpose(annotop)
    annot = {
        'annots': annots,
        'isKeyframe': False
    }
    return annot


def mvposev1(dataset, args, cfg, annotations, group):
    group.clear()
    annots = annotations
    affinity, dimGroups = affinity_model(annots)
    group = simple_associate(annots, affinity, dimGroups, dataset.Pall, group, cfg=cfg.associate) #NEM AD VISSSSZAA SEMMIT <- faszt nem, csak 3 kameránál csak 2%ban, akkor is csak egy embert...
    results = group

    resultslist = []
    for pid, people in results.items():
        result = {'id': pid, 'keypoints3d': people.keypoints3d}
        resultslist.append(result)
    return resultslist
#endregion

    #EasyMocapTrack3DFunctions
#region EasyMocapTrack3DFunctions

def compute_dist(results, tracker):
        edges = {}
        window_size = tracker.WINDOW_SIZE
        results_window = results
        dimGroups, frames = getDimGroups(results_window)
        dist = tracker._compute_dist(dimGroups, results_window)
        res = matchSVT(dist, dimGroups, control=tracker.svt_args)
        xx, yy = np.where(res)
        for x, y in zip(xx, yy):
            if x >= y:continue
            nf0, nf1 = frames[x], frames[y]
            ni0, ni1 = x - dimGroups[nf0], y - dimGroups[nf1]
            edge = ((nf0, ni0), (nf1, ni1))
            if edge not in edges:
                edges[edge] = []
            edges[edge].append(res[x, y])
        return edges

#endregion 


if __name__ == "__main__":
    # Import Openpose (Windows/Ubuntu/OSX)
    dir_path = os.path.dirname(os.path.realpath(__file__))
    try:
        # Windows Import
        if platform == "win32":
            # Change these variables to point to the correct folder (Release/x64 etc.)
            sys.path.append(dir_path + '/../../python/openpose/Release');
            os.environ['PATH']  = os.environ['PATH'] + ';' + dir_path + '/../../x64/Release;' +  dir_path + '/../../bin;'
            import pyopenpose as op
        else:
            # Change these variables to point to the correct folder (Release/x64 etc.)
            sys.path.append('../../python');
            # If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the OpenPose/python module from there. This will install OpenPose and the python library at your desired installation path. Ensure that this is in your python path in order to use it.
            # sys.path.append('/usr/local/python')
            from openpose import pyopenpose as op
    except ImportError as e:
        print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
        raise e


    # Custom Params (refer to include/openpose/flags.hpp for more parameters)
    params = dict()
    params["model_folder"] = "../../../models/"
    params["net_resolution"] = '-1x176'

    #params["display"] = 0
    #params["face"] = True
    #params["hand"] = True


    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

        #mvmpinit
    #region mvmpinit

    

    from easymocap.mytools import load_parser, parse_parser
    parser = load_parser()
    parser.add_argument('--vis_match', action='store_true')
    parser.add_argument('--time', action='store_true')
    parser.add_argument('--vis3d', action='store_true')
    parser.add_argument('--ret_crop', action='store_true')
    parser.add_argument('--no_write', action='store_true')
    parser.add_argument("--host", type=str, default='none')  # cn0314000675l
    parser.add_argument("--port", type=int, default=9999)
    args = parse_parser(parser)
    from easymocap.config.mvmp1f import Config
    cfg = Config.load(args.cfg, args.cfg_opts)
    # Define dataset
    help="""
  Demo code for multiple views and one person:

    - Input : {} => {}
    - Output: {}
    - Body  : {}
""".format(args.path, ', '.join(args.sub), args.out, 
    args.body)
    print(help)
    from easymocap.dataset import MVMPMF
    dataset = MVMPMF(args.path, cams= args.sub, annot_root=args.annot,
        config=CONFIG[args.body], kpts_type=args.body,
        undis=args.undis, no_img=True, out=args.out, filter2d=cfg.dataset)
    #dataset.no_img = not (args.vis_det or args.vis_match or args.vis_repro or args.ret_crop)
    noimg = True
    #start, end = args.start, min(args.end, len(dataset))
    affinity_model = ComposedAffinity(cameras=dataset.cameras, basenames=dataset.cams, cfg=cfg.affinity)
    group = PeopleGroup(Pall=dataset.Pall, cfg=cfg.group)



    #endregion

    #start visualization server
    #run in cmd:
    #py {Easymocap path}/apps/vis/vis_server.py --cfg config/vis3d/o3d_scene.yml
    visstarted = True
    try:
        client = BaseSocketClient('127.0.0.1', 9999)
    except :
        visstarted = False
        print("Visualization server not opened")

    #create tracker object
    winsize = 3
    tracker = Track3D(with2d=False, path = None, out = None, WINDOW_SIZE = winsize, MIN_FRAMES = 3, SMOOTH_SIZE = 0 )


    #init
    numc = 3 #number of cameras
    caps = [] #array of captures
    datums = [] #array of received data
    framecount = 0
    goodframes = 0
    idlist = [-1] #person id
    Resultlist_multiframe = []
    fromwebcams = False

    #fill arrays
    
    for i in range(1,numc+1):
        caps.append(cv2.VideoCapture(i)) 
        datums.append(op.Datum())
    if fromwebcams:
        if not caps[0].isOpened():
            print("Cannot open camera")
            exit()

    #execute
    try:
        start = timer()
        #while True:
        for frametoread in os.listdir(join(args.path, "images\\0")):
            frames = [] # array of captured frames in one time
            for i in range(numc):
                # Capture frame-by-frame
                #ret, frame = caps[i].read()
                #frame = cv2.resize(frame, (640,360))
                #height, width, _ = frame.shape
                #print("Height: " + str(height) + "\n" + "Width: " + str(width))
                

                #cv2.imshow('image' + str(i) +" cam", frame)
                readstr = join(args.path, "images" , dataset.cams[i] , frametoread)
                #print(readstr)
                frame = cv2.imread(readstr)
                #height, width, _ = frame.shape
                #print("Height: " + str(height) + "\n" + "Width: " + str(width))
                #cv2.imshow('image' + str(i), frame)
                #cv2.waitKey(0) #nem ugyanaz a két megjelenített frame felbontása, ezért baszódik el valószínűleg a 3D reconstruction

                frames.append(frame)
                #cv2.imshow('image' + str(i) , frame)
                #cv2.waitKey(0)
                #print(ret)
                # if frame is read correctly ret is True
                #if not ret:
                #    print(f"Can't receive frame from {i}th camera (stream end?). Exiting ...")
                #    break
            for i, frame in enumerate(frames):
                datums[i].cvInputData = frame
                #cv2.imshow('image' + str(i) , datums[i].cvInputData)
                #cv2.waitKey(0)
                opWrapper.emplaceAndPop(op.VectorDatum([datums[i]]))
                #print(f"Body keypoints from the {i}th camera: \n" + str(datums[i].poseKeypoints))
                #print("Type: " + str(type(datums[i].poseKeypoints)))   #type(datums.[i].poseKeypoints) -> numpy.ndarray
                #print("Shape: " + str(datums[i].poseKeypoints.shape))  #datums[i].poseKeypoints.shape -> (numPeople, numJoints, 3(x,y,conf))
                #break
            framecount += 1
            annots_allcam = []
            for i , datum in enumerate(datums):
                personlist = []
                for person in datum.poseKeypoints:
                    flattened = person.flatten().tolist()
                    personlist.append({"person_id": idlist , "pose_keypoints_2d" : flattened})
                annot = {"people" : personlist}
                annotconv = convert_from_openpose(annot)['annots']

                #Filter
                if dataset.filter2d is not None:
                    annot_valid = []
                    for ann in annotconv:
                        if dataset.filter2d(**ann):
                            annot_valid.append(ann)
                        #    print("appended")
                        #else:
                        #    print("not appended")
                    annotconv = annot_valid
                    annotconv = dataset.filter2d.nms(annotconv)

                annots_allcam.append(annotconv)

            #Get 3D keypoints 
            results = mvposev1(dataset, args, cfg, annots_allcam, group)
            print("Resultlist: " , results)    #PRINT RESULTS

            ##Track across frames
            #Resultlist_multiframe.append(results)
            #if framecount > winsize:
            #    Resultlist_multiframe.pop(0)
            #if len(Resultlist_multiframe) == winsize:
            #    edges = compute_dist(Resultlist_multiframe, tracker)
            #    print('Edges: ' + str(edges))
            #    trackedresults = tracker.associate(Resultlist_multiframe, edges)
            if visstarted:
                client.send(results)

            #Ha mind a 4 embert megtalálta az adott frame-n
            if len(results) == 4:
                goodframes +=1
        print(f"Framecount: {framecount}")
        end = timer()
        elapsed = end - start
        print(f"Time to execute: {elapsed}")
        fps = framecount / elapsed
        print(f"FPS: {fps}")
        percent = (goodframes / float(framecount))*100
        print(f"3Dfound rate: {percent}percent")
        for cap in caps:
            cap.release()
    # when done
    except KeyboardInterrupt:
        print(f"Framecount: {framecount}")
        end = timer()
        elapsed = end - start
        print(f"Time to execute: {elapsed}")
        fps = framecount / elapsed
        print(f"FPS: {fps}")
        percent = (goodframes / float(framecount))*100
        print(f"3Dfound rate: {percent}percent")
        for cap in caps:
            cap.release()