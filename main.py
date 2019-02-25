import os
import glob
import numpy as np


def get_filenames(root_dir, file_ending):
    filelist = []
    for root, dirs, files in os.walk(root_dir):
        for name in glob.glob(os.path.join(root,'*'+file_ending+'.pcd')):
                filelist.append(name.replace(file_ending+'.pcd', ''))
    return filelist


def get_pair_names(files_dir, filename):
    fll = os.path.join(files_dir, filename + 'FLL.pcd')
    flr = os.path.join(files_dir, filename + 'FRL.pcd')
    return fll, flr


def get_FLL2FLC_matrix(yaml_filename, vehicle_ID):
    # TODO: this should be extracted from yaml file! I'm hard coding it
    if vehicle_ID == "iceman":
        # ICEMAN config
        mat = np.array([[ 0.996718,  -0.0147415 ,0.0795959, -0.34114],
                       [0.00435741,  0.991623, 0.129089, -0.209279],
                       [-0.0808322, -0.128318, 0.988433, -0.31865],
                       [0, 0, 0, 1]])
    elif vehicle_ID == "goose":
        mat = np.array([[0.98396945, -0.0017633, 0.17832822, -0.33731908],
                        [-0.025881818, 0.98795247, 0.15257807, -0.20464729],
                        [-0.17644885, -0.15474762, 0.97206944, -0.1317835],
                        [0, 0, 0, 1]])
    else:
        raise Exception("vehicle_ID must be either goose or iceman, not ", vehicle_ID)

    return mat


def get_FLR2FLC_matrix(yaml_filename, vehicle_ID):
    # TODO: this should be extracted from yaml file! I'm hard coding it
    if vehicle_ID == "iceman":
        mat = np.array([[0.985045, -0.00689032, 0.17216, 0.627352],
                        [-0.0207143, 0.987217, 0.158032, -0.197764],
                        [-0.171048, -0.159234, 0.97231, -0.357467],
                        [0, 0, 0, 1]])
    elif vehicle_ID == "goose":
        mat = np.array([[0.95861739, -0.019034132, 0.28406063, 0.63547355],
                        [-0.03420715, 0.98283356, 0.18129565, -0.21253721],
                        [-0.28263512, -0.18351005, 0.9415102, -0.33163452],
                        [0, 0, 0, 1]])
    else:
        raise Exception("vehicle_ID must be either goose or iceman, not ", vehicle_ID)
    return mat


def count_num_point(filename):
    count = 0
    for line in open(filename):
        line = line.strip().split(' ')
        if line[0] not in ["#", "VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "VIEWPOINT", "POINTS",
                           "DATA"]:
            count += 1
    return count

def convert_Lidar2fused(line, filename, vehicle_ID):
    line = line.strip().split(' ')
    if line[0] not in ["#", "VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "VIEWPOINT", "POINTS",
                       "DATA"]:
        x = float(line[0])
        y = float(line[1])
        z = float(line[2])

        if "FLL" in filename:
            transform = get_FLL2FLC_matrix("",vehicle_ID)
        elif "FRL" in filename:
            transform = get_FLR2FLC_matrix("",vehicle_ID)
        else:
            raise Exception("Lidar Sensor Unknown, must be either FLL or FRL")

        flc_coordinates = transform.dot(np.array([x,y,z, 1.0]))

        return flc_coordinates


def add_on_top(filename, num_points):
    if os.path.exists(filename):
        os.remove(filename)

    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format \n")
        f.write("VERSION 0.7 \n")
        f.write("FIELDS x y z intensity ring \n")
        f.write("SIZE 4 4 4 4 2 \n")
        f.write("TYPE F F F F U \n")
        f.write("COUNT 1 1 1 1 1 \n")
        f.write("WIDTH " + str(num_points) + " \n")
        f.write("HEIGHT 1 \n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0 \n")
        f.write("POINTS " + str(num_points) + " \n")
        f.write("DATA ascii \n")


def test(pcd_files_dir, vehicle_ID):
    filelist = get_filenames(pcd_files_dir, file_ending='FLL')

    for i, filename in enumerate(filelist):
        filename1 , filename2 = get_pair_names(pcd_files_dir, filename)

        fused_filename = os.path.join(pcd_files_dir, filename + 'fused.pcd')

        # Create header and account for number of points in a pcd file
        num_points = count_num_point(filename1) + count_num_point(filename2)
        add_on_top(fused_filename, num_points)

        with open(fused_filename, 'a') as fused:
            for line in open(filename1):
                coord = convert_Lidar2fused(line, filename1,vehicle_ID)
                if coord is not None:
                    new_line = ' '.join(str(i) for i in coord[0:3]) + ' ' + str(line[3]) + ' ' + str(line[4]) + '\n'
                    fused.write(new_line)

            for line in open(filename2):
                coord = convert_Lidar2fused(line, filename2,vehicle_ID)
                if coord is not None:
                    new_line = ' '.join(str(i) for i in coord[0:3]) + ' ' + str(line[3]) + ' ' + str(line[4]) + '\n'
                    fused.write(new_line)


if __name__ == "__main__":
    path = "/Users/elham.s/SSIC/Codes/Data/selected4annot/2019-01-23-15-17/"
    test(path, vehicle_ID="goose")







