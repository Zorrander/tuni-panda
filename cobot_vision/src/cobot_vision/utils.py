def convert_detection_pose(x ,y):
    listener_tf = tf.TransformListener()
    camera_focal = 550
    (trans1, rot1) = listener_tf.lookupTransform('/panda_link0', '/camera_color_frame', rospy.Time(0))
    z_to_surface = trans1[2]
    to_world_scale = z_to_surface / camera_focal

    x_dist = x * to_world_scale
    y_dist = y * to_world_scale

    my_point = PoseStamped()
    my_point.header.frame_id = "camera_color_frame"
    my_point.header.stamp = rospy.Time(0)
    my_point.pose.position.x = 0
    my_point.pose.position.y = -x_dist
    my_point.pose.position.z = y_dist
    theta = 0
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    my_point.pose.orientation.x = quat[0]
    my_point.pose.orientation.y = quat[1]
    my_point.pose.orientation.z = quat[2]
    my_point.pose.orientation.w = quat[3]
    ps = listener_tf.transformPose("/panda_link0", my_point)

    (trans, rot) = listener_tf.lookupTransform('/panda_link0', '/camera_color_frame', rospy.Time(0))
    data = (ps.pose.position.x - trans[0], ps.pose.position.y - trans[1])

    return [data[0], data[1]]


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]

def crop_rect(dims, crop_scale):

    dimensions=dims.copy()
    xy_dims=[abs(dimensions[2]-dimensions[0]), abs(dimensions[3]-dimensions[1])]

    max_edge = max(xy_dims)
    if crop_scale>=1:
        to_correct = int(max_edge*(crop_scale - 1)/2)
        dimensions[0] -= to_correct
        dimensions[1] -= to_correct
        dimensions[2] += to_correct
        dimensions[3] += to_correct
    else:
        to_correct = int(max_edge*(1 - crop_scale)/2)
        dimensions[0] += to_correct
        dimensions[1] += to_correct
        dimensions[2] -= to_correct
        dimensions[3] -= to_correct

    xy_dims=[abs(dimensions[2]-dimensions[0]), abs(dimensions[3]-dimensions[1])]
    xy_idx = np.argmax(xy_dims)
    half_dist=int(abs(xy_dims[1]-xy_dims[0])/2)

    New_x1=dimensions[0]-half_dist
    New_y1=dimensions[1]-half_dist
    New_x2=dimensions[2]+half_dist
    New_y2=dimensions[3]+half_dist

    if New_x1<0:
        New_x2+=abs(New_x1)
        New_x1=0

    if New_x2>640:
        New_x1-=abs(New_x2-640)
        New_x2=640
    if New_y1<0:
        New_y2+=abs(New_y1)
        New_y1=0

    if New_y2>480:
        New_y1-=abs(New_y2-480)
        New_y2=480

    max_edge = max(xy_dims)
    to_correct = max_edge*crop_scale

    if xy_idx == 0:
        return [dimensions[0], New_y1, dimensions[2], New_y2]
    else:
        return [New_x1, dimensions[1], New_x2, dimensions[3]]


def crop_img(img,crop_XYXY):

    return img[crop_XYXY[1]:crop_XYXY[3],crop_XYXY[0]:crop_XYXY[2],:]


def resize_img(img,size):

    raise NotImplementedError


def get_straight_bbox(img, rectangle):
    x, y, w, h, angle = rectangle[0], rectangle[1], rectangle[2], rectangle[3], -rectangle[4]
    cloned_img=img.copy()
    long_edge=int(max((w,h))/2*1.2)
    cropped_img = cloned_img[(y-long_edge):(y+long_edge), (x-long_edge):(x+long_edge),:]
    return cropped_img

def rot(phi):

  phi = np.deg2rad(phi)
  return np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])

def drawRect(img, rectangle):

    x, y, w, h, angle = rectangle[0], rectangle[1], rectangle[2], rectangle[3], -rectangle[4]
    cloned_img=img.copy()
    a = np.array((-w / 2, -h / 2))
    b = np.array((w / 2, -h / 2))
    c = np.array((w / 2, h / 2))
    d = np.array((-w / 2, h / 2))

    if angle != 0:
        a = np.matmul(rot(angle), a)
        b = np.matmul(rot(angle), b)
        c = np.matmul(rot(angle), c)
        d = np.matmul(rot(angle), d)

    a += [x, y]
    b += [x, y]
    c += [x, y]
    d += [x, y]
    a=a.astype(int)
    b=b.astype(int)
    c=c.astype(int)
    d=d.astype(int)

    cv2.line(cloned_img, (a[0],a[1]), (b[0],b[1]), (0,255,0), 2)
    cv2.line(cloned_img, (b[0],b[1]), (c[0],c[1]), (0,255,0), 2)
    cv2.line(cloned_img, (c[0],c[1]), (d[0],d[1]), (0,255,0), 2)
    cv2.line(cloned_img, (d[0],d[1]), (a[0],a[1]), (0,255,0), 2)

    return cloned_img