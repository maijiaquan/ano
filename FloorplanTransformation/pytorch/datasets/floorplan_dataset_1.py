from torch.utils.data import Dataset

import numpy as np
import time

#from plane_dataset_scannet import PlaneDatasetScanNet
#from augmentation import *
from utils import *
from skimage import measure
import cv2
import copy


def lineRange2(line):
    direction = calcLineDirection(line)
    if direction < 2:
        fixedValue = (line[0][1 - direction] + line[1][1 - direction]) // 2
        minValue = min(line[0][direction], line[1][direction])
        maxValue = max(line[0][direction], line[1][direction])
        return direction, True, fixedValue, minValue, maxValue
    else:
        return direction, False, None, None, None

def lineRange(line):
    direction = calcLineDirection(line)
    fixedValue = (line[0][1 - direction] + line[1][1 - direction]) // 2
    minValue = min(line[0][direction], line[1][direction])
    maxValue = max(line[0][direction], line[1][direction])
    return direction, fixedValue, minValue, maxValue

def pointDistance(point_1, point_2):
    return max(abs(point_1[0] - point_2[0]), abs(point_1[1] - point_2[1]))

# Euclidean distance between two points
def p2p(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

# distance between point and line
def p2l(p, l):
    return min(pointDistance(p, l[i]) for i in range(2))

# distance between line and line
def l2l(l1, l2):
    return min(pointDistance(l1[i], l2[j]) for i in range(2) for j in range(2))

# intersection point between two lines
def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False

# check if point in a line
def pInLine(p, l, gap):
    return abs(p2p(p, l[0]) + p2p(p, l[1]) - p2p(*l)) < gap*2

def divideWalls(walls):
    horizontalWalls = []
    verticalWalls = []
    for wall in walls:
        if calcLineDirection(wall) == 0:
            horizontalWalls.append(wall)
        else:
            verticalWalls.append(wall)
            pass
        continue
    return horizontalWalls, verticalWalls

def mergeLines(line_1, line_2):
    direction_1, isMan_1, fixedValue_1, min_1, max_1 = lineRange(line_1)
    direction_2, _, fixedValue_2, min_2, max_2 = lineRange(line_2)
    if isMan_1:
        fixedValue = (fixedValue_1 + fixedValue_2) // 2
        if direction_1 == 0:
            return [(min(min_1, min_2), fixedValue), (max(max_1, max_2), fixedValue)]
        else:
            return [(fixedValue, min(min_1, min_2)), (fixedValue, max(max_1, max_2))]
    else:
        if direction_1 == 3:
            return [(min(line_1[0][0], line_2[1][0]), min(line_1[0][1], line_2[1][1])), 
                    (min(line_1[0][0], line_2[1][0]), min(line_1[0][1], line_2[1][1]))]
        else:
            return [(max(line_1[0][0], line_2[1][0]), min(line_1[0][1], line_2[1][1])), 
                    (min(line_1[0][0], line_2[1][0]), max(line_1[0][1], line_2[1][1]))]

def findConnections(line_1, line_2, gap):

    isMan_1, isMan_2 = isManhattan(line_1), isManhattan(line_2)

    if True or isMan_1 and isMan_2:

        connection_1 = -1
        connection_2 = -1
        pointConnected = False
        for c_1 in range(2):
            if pointConnected:
                break
            for c_2 in range(2):
                if pointDistance(line_1[c_1], line_2[c_2]) > gap:
                    continue

                connection_1 = c_1
                connection_2 = c_2
                connectionPoint = ((line_1[c_1][0] + line_2[c_2][0]) // 2, (line_1[c_1][1] + line_2[c_2][1]) // 2)
                pointConnected = True
                break
            continue
        if pointConnected:
            return [connection_1, connection_2], connectionPoint
        #direction_1, isMan_1, fixedValue_1, min_1, max_1 = lineRange(line_1)
        #direction_2, isMan_2, fixedValue_2, min_2, max_2 = lineRange(line_2)

        direction_1, fixedValue_1, min_1, max_1 = lineRange(line_1)
        direction_2, fixedValue_2, min_2, max_2 = lineRange(line_2)


        if direction_1 == direction_2:
            return [-1, -1], (0, 0)

        if min(fixedValue_1, max_2) < max(fixedValue_1, min_2) - gap or min(fixedValue_2, max_1) < max(fixedValue_2, min_1) - gap:
            return [-1, -1], (0, 0)
        if abs(min_1 - fixedValue_2) <= gap:
            return [0, 2], (fixedValue_2, fixedValue_1)
        if abs(max_1 - fixedValue_2) <= gap:
            return [1, 2], (fixedValue_2, fixedValue_1)
        if abs(min_2 - fixedValue_1) <= gap:
            return [2, 0], (fixedValue_2, fixedValue_1)
        if abs(max_2 - fixedValue_1) <= gap:
            return [2, 1], (fixedValue_2, fixedValue_1)
        return [2, 2], (fixedValue_2, fixedValue_1)
    else:
        pass
        def p2d(c, ps):
            d = []
            for p in ps:
                direction = calcLineDirection((p, c))
                if direction == 0:
                    d.append(2 if p[0] > c[0] else 0)
                elif direction == 1:
                    d.append(1 if p[1] > c[1] else 3)
                elif direction == 2:
                    d.append(6 if p[0] > c[0] and p[1] > c[1] else 4)
                else:
                    d.append(7 if p[0] > c[0] and p[1] < c[1] else 5)
            return d

        #print('-'*20, line_1, line_2)
        l1, l2 = line(*line_1), line(*line_2)
        p = intersection(l1, l2)
        w1, w2 = pInLine(p, line_1, gap), pInLine(p, line_2, gap)

        if not w1 and not w2 and l2l(line_1, line_2) < gap:
            return p2d(p, [line_1[0], line_2[0]]), p
        elif w1 and not w2 and p2l(p, line_2) < gap:
            return p2d(p, [line_1[0], line_1[1], line_2[0]]), p
        elif not w1 and w2 and p2l(p, line_1) < gap:
            return p2d(p, [line_1[0], line_2[1], line_2[0]]), p
        elif w1 and w2:
            ps = [l[i] for l in [line_1, line_2] for i in range(2) if pointDistance(p, l[i]) >= gap]
            assert len(ps) > 1, 'Number of points is not enough !'
            return p2d(p, ps), p

        return [-1, -1], (0, 0)

def lines2Corners(lines, gap):
    success = True

    connectionCornerMap = {}

    # Manhattan patterns (with comment)
    connectionCornerMap[(1, 1)] = 4 # (0, 3)
    connectionCornerMap[(0, 1)] = 5 # (0, 1)
    connectionCornerMap[(0, 0)] = 6 # (1, 2)
    connectionCornerMap[(1, 0)] = 7 # (2, 3)

    # slant patterns
    connectionCornerMap[(2, 4)] = 8
    connectionCornerMap[(3, 5)] = 9
    connectionCornerMap[(0, 6)] = 10
    connectionCornerMap[(1, 7)] = 11

    connectionCornerMap[(2, 5)] = 12
    connectionCornerMap[(3, 6)] = 13
    connectionCornerMap[(0, 7)] = 14
    connectionCornerMap[(1, 4)] = 15

    connectionCornerMap[(5, 6)] = 16
    connectionCornerMap[(6, 7)] = 17
    connectionCornerMap[(4, 7)] = 18
    connectionCornerMap[(4, 5)] = 19

    connectionCornerMap[(0, 5)] = 20
    connectionCornerMap[(2, 6)] = 21
    connectionCornerMap[(2, 7)] = 22
    connectionCornerMap[(0, 4)] = 23
 
    connectionCornerMap[(3, 4)] = 24
    connectionCornerMap[(3, 7)] = 25
    connectionCornerMap[(1, 5)] = 26
    connectionCornerMap[(1, 6)] = 27

    connectionCornerMap[(2, 0)] = 28 # (1, 2, 3)
    connectionCornerMap[(1, 2)] = 29 # (0, 2, 3)
    connectionCornerMap[(2, 1)] = 30 # (0, 1, 3)
    connectionCornerMap[(0, 2)] = 31 # (0, 1, 2)

    connectionCornerMap[(0, 2, 4)] = 32
    connectionCornerMap[(0, 2, 7)] = 33
    connectionCornerMap[(0, 2, 5)] = 34
    connectionCornerMap[(0, 2, 6)] = 35

    connectionCornerMap[(2, 4, 6)] = 36
    connectionCornerMap[(2, 5, 7)] = 37
    connectionCornerMap[(0, 5, 7)] = 38
    connectionCornerMap[(0, 4, 6)] = 39

    connectionCornerMap[(1, 3, 4)] = 40
    connectionCornerMap[(1, 3, 5)] = 41
    connectionCornerMap[(1, 3, 7)] = 42
    connectionCornerMap[(1, 3, 6)] = 43

    connectionCornerMap[(1, 4, 6)] = 44
    connectionCornerMap[(1, 5, 7)] = 45
    connectionCornerMap[(3, 5, 7)] = 46
    connectionCornerMap[(3, 4, 6)] = 47

    connectionCornerMap[(4, 5, 7)] = 48
    connectionCornerMap[(5, 6, 7)] = 49
    connectionCornerMap[(4, 6, 7)] = 50
    connectionCornerMap[(4, 5, 6)] = 51

    connectionCornerMap[(2, 2)] = 52 # (0, 1, 2, 3)
    connectionCornerMap[(1, 3, 4, 6)] = 53
    connectionCornerMap[(1, 3, 5, 7)] = 54
    connectionCornerMap[(0, 2, 4, 6)] = 55
    connectionCornerMap[(0, 2, 5, 7)] = 56
    connectionCornerMap[(4, 5, 6, 7)] = 57


    corners = []
    for lineIndex_1 in range(len(lines)-1):
        for lineIndex_2 in range(lineIndex_1+1, len(lines)):
            line_1, line_2 = lines[lineIndex_1], lines[lineIndex_2]

            if calcLineDirection(line_1) == calcLineDirection(line_2):
                continue
            
            connections, connectionPoint = findConnections(line_1, line_2, gap=gap)
            if connections[0] == -1 and connections[1] == -1:
                continue
            if not isManhattan(line_1) or not isManhattan(line_2):
                connections = sorted(connections)
            corners.append((connectionPoint, connectionCornerMap[tuple(connections)]))
            continue
    print(corners)
    return corners, success

def getRoomLabelMap():
    labelMap = {}
    labelMap['living_room'] = 1
    labelMap['kitchen'] = 2
    labelMap['bedroom'] = 3
    labelMap['bathroom'] = 4
    labelMap['restroom'] = 4
    labelMap['washing_room'] = 4    
    labelMap['office'] = 3
    labelMap['closet'] = 6
    labelMap['balcony'] = 7
    labelMap['corridor'] = 8
    labelMap['dining_room'] = 9
    labelMap['laundry_room'] = 10
    labelMap['PS'] = 10    
    return labelMap

def getIconLabelMap():
    labelMap = {}
    labelMap['bathtub'] = 1
    labelMap['cooking_counter'] = 2
    labelMap['toilet'] = 3
    labelMap['entrance'] = 4
    labelMap['washing_basin'] = 5
    labelMap['special'] = 6
    labelMap['stairs'] = 7
    return labelMap

def loadLabelMap():
    roomMap = getRoomLabelMap()
    iconMap = getIconLabelMap()

    labelMap = {}
    for icon, label in iconMap.items():
        labelMap[icon] = ('icons', label)
        continue
    for room, label in roomMap.items():    
        labelMap[room] = ('rooms', label)
        continue
    labelMap['door'] = 8
    return labelMap

def augmentSample(options, image, background_colors=[], split='train'):
    max_size = np.random.randint(low=int(options.width * 3 / 4), high=options.width + 1)
    if split != 'train':
        max_size = options.width
        pass
    image_sizes = np.array(image.shape[:2]).astype(np.float32)
    transformation = np.zeros((3, 3))
    transformation[0][0] = transformation[1][1] = float(max_size) / image_sizes.max()
    transformation[2][2] = 1
    image_sizes = (image_sizes / image_sizes.max() * max_size).astype(np.int32)
    
    if image_sizes[1] == options.width or split != 'train':
        offset_x = 0
    else:
        offset_x = np.random.randint(options.width - image_sizes[1])
        pass
    if image_sizes[0] == options.height or split != 'train':
        offset_y = 0
    else:
        offset_y = np.random.randint(options.height - image_sizes[0])
        pass

    transformation[0][2] = offset_x
    transformation[1][2] = offset_y

    if len(background_colors) == 0:
        full_image = np.full((options.height, options.width, 3), fill_value=255)
    else:
        full_image = background_colors[np.random.choice(np.arange(len(background_colors), dtype=np.int32), options.width * options.height)].reshape((options.height, options.width, 3))
        pass
        
    #full_image = np.full((options.height, options.width, 3), fill_value=-1, dtype=np.float32)
    full_image[offset_y:offset_y + image_sizes[0], offset_x:offset_x + image_sizes[1]] = cv2.resize(image, (image_sizes[1], image_sizes[0]))
    image = full_image

    if np.random.randint(2) == 0 and split == 'train':
        image = np.ascontiguousarray(image[:, ::-1])
        transformation[0][0] *= -1
        transformation[0][2] = options.width - transformation[0][2]
        pass
    return image, transformation

def convertToPoint(x, y):
    return (int(round(float(x))), int(round(float(y))))

def transformPoint(transformation, point):
    point = np.array(point)
    point = np.concatenate([point, np.ones(1)], axis=0)
    point = np.matmul(transformation, point)
    return tuple(np.round(point[:2] / point[2]).astype(np.int32).tolist())

## Plane dataset class
class FloorplanDataset(Dataset):
    def __init__(self, options, split, random=True):
        self.options = options
        self.split = split
        self.random = random
        self.imagePaths = []
        self.dataFolder = '../data/'
        with open(self.dataFolder + split + '.txt') as f:
            for line in f:
                self.imagePaths.append([value.strip() for value in line.split('\t')])
                continue
            
        if options.numTrainingImages > 0 and split == 'train':
            self.numImages = options.numTrainingImages
        else:
            self.numImages = len(self.imagePaths)            
            pass
        self.labelMap = loadLabelMap()        
        return
    
    def __len__(self):
        return self.numImages

    def __getitem__(self, index):
        if self.random:
            t = int(time.time() * 1000000)
            np.random.seed(((t & 0xff000000) >> 24) +
                           ((t & 0x00ff0000) >> 8) +
                           ((t & 0x0000ff00) << 8) +
                           ((t & 0x000000ff) << 24))
            index = np.random.randint(len(self.imagePaths))
        else:
            index = index % len(self.imagePaths)
            pass

        debug = -1
        if debug >= 0:
            index = debug
            print(index, self.imagePaths[index][1])
            pass
        
        image = cv2.imread(self.dataFolder + self.imagePaths[index][0])
        image_ori = image
        image_width, image_height = image.shape[1], image.shape[0]

        #def transformPoint(x, y, resize=False):
        #if resize:
        #return (int(round(float(x) * self.options.width / image_width)), int(round(float(y) * self.options.height / image_height)))
        #else:
        #return (int(round(float(x))), int(round(float(y))))            
        
        walls = []
        wall_types = []
        doors = []
        semantics = {}
        with open(self.dataFolder + self.imagePaths[index][1]) as info_file:
            line_index = 0
            for line in info_file:
                line = line.split('\t')
                label = line[4].strip()
                if label == 'wall':
                    walls.append((convertToPoint(line[0], line[1]), convertToPoint(line[2], line[3])))
                    wall_types.append(int(line[5].strip()) - 1)
                elif label in ['door', 'window']:
                    doors.append((convertToPoint(line[0], line[1]), convertToPoint(line[2], line[3])))
                else:
                    if label not in semantics:
                        semantics[label] = []
                        pass
                    semantics[label].append((convertToPoint(line[0], line[1]), convertToPoint(line[2], line[3])))
                    pass
                continue
            pass

        gap = 3
        #print(semantics)
        invalid_indices = {}
        for wall_index_1, (wall_1, wall_type_1) in enumerate(zip(walls, wall_types)):
            for wall_index_2, (wall_2, wall_type_2) in enumerate(zip(walls, wall_types)):
                if wall_type_1 == 0 and wall_type_2 == 1 and calcLineDirection(wall_1) == calcLineDirection(wall_2):
                    if min([pointDistance(wall_1[c_1], wall_2[c_2]) for c_1, c_2 in [(0, 0), (0, 1), (1, 0), (1, 1)]]) <= gap * 2:
                        walls[wall_index_1] = mergeLines(wall_1, wall_2)
                        invalid_indices[wall_index_2] = True
                        pass
                    pass
                continue
            continue
        walls = [wall for wall_index, wall in enumerate(walls) if wall_index not in invalid_indices]

        background_mask = measure.label(1 - drawWallMask(walls, image_width, image_height), background=0)
        wall_index = background_mask.min()
        background_colors = []
        if np.random.randint(2) == 0:
            for pixel in [(0, 0), (0, background_mask.shape[0] - 1), (background_mask.shape[1] - 1, 0), (background_mask.shape[1] - 1, background_mask.shape[0] - 1)]:
                index = background_mask[pixel[1]][pixel[0]]
                if index != wall_index:
                    background_colors = image[background_mask == index]
                    break
                continue
            pass
        
        #walls = connectWalls(walls, roomSegmentation, gap=gap)
        
        corners, success = lines2Corners(walls, gap=gap)
        if not success:
            #print('warning', index, self.imagePaths[index][1])
            pass


        if self.split == 'train':
            image, transformation = augmentSample(self.options, image, background_colors)
        else:
            image, transformation = augmentSample(self.options, image, background_colors, split=self.split)
            pass
        
        corners = [(transformPoint(transformation, corner[0]), corner[1]) for corner in corners]
        walls = [[transformPoint(transformation, wall[c]) for c in range(2)] for wall in walls]
        doors = [[transformPoint(transformation, door[c]) for c in range(2)] for door in doors]        
        for semantic, items in semantics.items():
            semantics[semantic] = [[transformPoint(transformation, item[c]) for c in range(2)] for item in items]
            continue

        width = self.options.width
        height = self.options.height
        
        roomSegmentation = np.zeros((height, width), dtype=np.uint8)
        for line in walls:
            #cv2.line(roomSegmentation, line[0], line[1], color=NUM_ROOMS + 1 + calcLineDirection(line), thickness=gap)
            cv2.line(roomSegmentation, line[0], line[1], color=NUM_ROOMS + 1, thickness=gap)
            continue

        rooms = measure.label(roomSegmentation == 0, background=0)
        
        corner_gt = []
        for corner in corners:
            corner_gt.append((corner[0][0], corner[0][1], corner[1] + 1))
            continue

        openingCornerMap = [[3, 1], [0, 2]]
        for opening in doors:
            direction = calcLineDirection(opening)
            for cornerIndex, corner in enumerate(opening):
                corner_gt.append((int(round(corner[0])), int(round(corner[1])), 14 + openingCornerMap[direction][cornerIndex]))
                continue
            continue

        wallIndex = rooms.min()
        for pixel in [(0, 0), (0, height - 1), (width - 1, 0), (width - 1, height - 1)]:
            backgroundIndex = rooms[pixel[1]][pixel[0]]
            if backgroundIndex != wallIndex:
                break
            continue
        iconSegmentation = np.zeros((height, width), dtype=np.uint8)
        for line in doors:
            cv2.line(iconSegmentation, line[0], line[1], color = self.labelMap['door'], thickness=gap - 1)
            continue

        roomLabelMap = {}
        for semantic, items in semantics.items():
            group, label = self.labelMap[semantic]
            for corners in items:
                if group == 'icons':
                    if label == 0:
                        continue
                    cv2.rectangle(iconSegmentation, (int(round(corners[0][0])), int(round(corners[0][1]))), (int(round(corners[1][0])), int(round(corners[1][1]))), color=label, thickness=-1)
                    corner_gt.append((corners[0][0], corners[0][1], 18 + 2))
                    corner_gt.append((corners[0][0], corners[1][1], 18 + 1))
                    corner_gt.append((corners[1][0], corners[0][1], 18 + 3))
                    corner_gt.append((corners[1][0], corners[1][1], 18 + 0))
                else:
                    roomIndex = rooms[(corners[0][1] + corners[1][1]) // 2][(corners[0][0] + corners[1][0]) // 2]
                    if roomIndex == wallIndex or roomIndex == backgroundIndex:
                        #if roomIndex == backgroundIndex:
                        #print('label on background', corners, semantic, index, self.imagePaths[index][1])
                        #pass
                        continue
                    if roomIndex in roomLabelMap:
                        #print('room has more than one labels', corners, label, roomLabelMap[roomIndex])
                        #exit(1)
                        continue
                        pass
                    roomLabelMap[roomIndex] = label
                    roomSegmentation[rooms == roomIndex] = label
                    pass
                continue
            continue
        
        #print(roomLabelMap)
        if debug >= 0:
            cv2.imwrite('test/floorplan/rooms.png', drawSegmentationImage(rooms, blackIndex=backgroundIndex))
            exit(1)
            pass
        
        for roomIndex in range(rooms.min(), rooms.max() + 1):
            if roomIndex == wallIndex or roomIndex == backgroundIndex:
                continue
            if roomIndex not in roomLabelMap:
                roomSegmentation[rooms == roomIndex] = 10
                #print('room has no label', roomIndex, rooms.max(), np.stack((rooms == roomIndex).nonzero(), axis=-1).mean(0)[::-1])
                #exit(1)
                pass
            continue

        cornerSegmentation = np.zeros((height, width, 66), dtype=np.uint8)
        for corner in corner_gt:
            cornerSegmentation[min(max(corner[1], 0), height - 1), min(max(corner[0], 0), width - 1), corner[2] - 1] = 1
            continue

        if False:
            cv2.imwrite('test/image.png', image_ori)
            cv2.imwrite('test/icon_segmentation.png', drawSegmentationImage(iconSegmentation))
            cv2.imwrite('test/room_segmentation.png', drawSegmentationImage(roomSegmentation))
            cv2.imwrite('test/corner_segmentation.png', drawSegmentationImage(cornerSegmentation, blackIndex=0))
            print([(seg.min(), seg.max(), seg.shape) for seg in [cornerSegmentation, iconSegmentation, roomSegmentation]])
            exit(1)            
            pass


        image = (image.astype(np.float32) / 255 - 0.5).transpose((2, 0, 1))
        kernel = np.zeros((3, 3), dtype=np.uint8)
        kernel[1] = 1
        kernel[:, 1] = 1
        cornerSegmentation = cv2.dilate(cornerSegmentation, kernel, iterations=5)

        sample = [image, cornerSegmentation.astype(np.float32), iconSegmentation.astype(np.int64), roomSegmentation.astype(np.int64)]
        return sample
