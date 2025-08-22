#!/usr/bin/env python3
import sys
import os.path
import orbslam3
import time
import cv2

def main(vocab_path, settings_path, path_to_image_folder, path_to_times_file):
    image_filenames, timestamps = load_images(path_to_image_folder, path_to_times_file)
    num_images = len(image_filenames)

    slam = orbslam3.System(vocab_path, settings_path, orbslam3.Sensor.MONOCULAR)
    # The viewer is not implemented in this Python binding, so set to False
    slam.set_use_viewer(False) 
    slam.initialize()

    times_track = [0 for _ in range(num_images)]
    print('-----')
    print('Start processing sequence ...')
    print('Images in the sequence: {0}'.format(num_images))

    # Main loop
    for idx in range(num_images):
        image = cv2.imread(image_filenames[idx], cv2.IMREAD_UNCHANGED)
        tframe = timestamps[idx]

        if image is None:
            print("failed to load image at {0}".format(image_filenames[idx]))
            return 1
        
        # Add a check for RGB images
        if len(image.shape) > 2:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        t1 = time.time()
        slam.process_image_mono(image, tframe)
        t2 = time.time()

        ttrack = t2 - t1
        times_track[idx] = ttrack

        t = 0
        if idx < num_images - 1:
            t = timestamps[idx + 1] - tframe
        elif idx > 0:
            t = tframe - timestamps[idx - 1]

        if ttrack < t:
            time.sleep(t - ttrack)

    # Save trajectory
    save_trajectory(slam.get_trajectory_points(), 'trajectory.txt')

    slam.shutdown()

    times_track = sorted(times_track)
    total_time = sum(times_track)
    print('-----')
    print('median tracking time: {0}'.format(times_track[num_images // 2]))
    print('mean tracking time: {0}'.format(total_time / num_images))

    return 0

def load_images(path_to_images, path_to_times_file):
    """
    Corrected function to load EuRoC dataset images.
    The times file is data.csv which contains timestamp and filename.
    """
    image_files = []
    timestamps = []
    with open(path_to_times_file, 'r') as times_file:
        for line in times_file.readlines():
            if line[0] == '#':
                continue
            
            line_parts = line.rstrip().split(',')
            # EuRoC timestamp is the first column, in nanoseconds
            ts = float(line_parts[0]) / 1e9 
            # EuRoC image filename is the second column
            img_name = line_parts[1]
            
            timestamps.append(ts)
            image_files.append(os.path.join(path_to_images, img_name))
            
    return image_files, timestamps

def save_trajectory(trajectory, filename):
    """
    Corrected function to save trajectory from a list of (timestamp, pose_matrix) tuples.
    """
    with open(filename, 'w') as traj_file:
        for timestamp, pose in trajectory:
            # Flatten the 4x4 pose matrix into a single line for KITTI format
            p = pose.flatten()
            line = "{time} {p0} {p1} {p2} {p3} {p4} {p5} {p6} {p7} {p8} {p9} {p10} {p11}\n".format(
                time=repr(timestamp),
                p0=repr(p[0]), p1=repr(p[1]), p2=repr(p[2]), p3=repr(p[3]),
                p4=repr(p[4]), p5=repr(p[5]), p6=repr(p[6]), p7=repr(p[7]),
                p8=repr(p[8]), p9=repr(p[9]), p10=repr(p[10]), p11=repr(p[11]),
            )
            traj_file.write(line)

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print('Usage: ./test_mono.py path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file')
        sys.exit(1)
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])