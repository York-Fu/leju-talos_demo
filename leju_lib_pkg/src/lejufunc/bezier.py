from motion.TrajectoryPlan import TrajectoryPlanning


MIN_INTERVAL = 10.0

def points_2_frames(points, interval=MIN_INTERVAL):
    results = [[] for _ in range(len(points[0]))]
    for point in points:
        for i in range(len(point)):
            results[i].append(point[i].y)
    return results


def get_bezier_frames(p0, p1, p2, act_time, frame_time=MIN_INTERVAL):
    tpObject = TrajectoryPlanning(len(p0), frame_time)
    tpObject.setInterval(act_time)
    tpObject.planningBegin(p0, p1)
    if p2 == None:
        results = points_2_frames(tpObject.planningEnd())
    else:
        results = points_2_frames(tpObject.planning(p2))
    return results


def generator_bezier(poseList):
    tpObject = TrajectoryPlanning(22, 10.0)
    interval = poseList[1][1] if poseList[1][1] > MIN_INTERVAL else MIN_INTERVAL
    tpObject.setInterval(interval)
    tpObject.planningBegin(poseList[0][0], poseList[1][0])
    for poseIndex in range(2, len(poseList)):
        interval = poseList[poseIndex][1] if poseList[poseIndex][1] > MIN_INTERVAL else MIN_INTERVAL
        tpObject.setInterval(interval)
        trajectoryPoint = tpObject.planning(poseList[poseIndex][0])
        yield points_2_frames(trajectoryPoint), poseList[poseIndex-1][2] / 1000.0, False

    trajectoryPoint = tpObject.planningEnd()
    yield points_2_frames(trajectoryPoint), poseList[-1][2] / 1000.0, True


# auto bezier example code:
if __name__ == '__main__':
    poseList = [
        [0,-1,21,-40,-18,-1,0,1,-21,40,18,1,-16,-58,-21,16,58,21,0,0,0,0],
        [0,-1,21,-40,-18,-1,0,1,-21,40,18,1,-16,-58,-21,16,58,21,0,0,0,0],
    ]
    start = [0,-1,21,-40,-18,-1,0,1,-21,40,18,1,-16,-58,-21,16,58,21,0,0,0,0]
    stop = [0,-1,21,-40,-18,-1,0,1,-21,40,18,1,-16,-58,-21,16,58,21,0,0,0,0]
    for pos in get_bezier_frames(start, stop, 100):
        print(pos)