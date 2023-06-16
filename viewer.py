import cv2
import numpy as np
from kinect import Kinect


def main():
    cam = Kinect()
    cam.start(size=1536)

    while True:
        if cam.isReady:
            try:
                color, depth = cam.get_data()
                depth = cv2.applyColorMap(depth.astype(np.uint8), cv2.COLORMAP_JET, dst=1)
                color = cv2.resize(color, dsize=[800, 600])
                depth = cv2.resize(depth, dsize=[800, 600])
                # imu = cam.get_imu()
                # print(imu)
                # pose  = cam.get_pose()
                # print(pose)
            except Exception as e:
                print(e)
                continue

            img = np.hstack((color[:, :, ::-1], depth))

            cv2.imshow('color', img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
