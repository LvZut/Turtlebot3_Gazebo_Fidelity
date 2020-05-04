import os
import os.path


def main():

    script_dir = os.path.dirname(__file__)
    rel_path = "../logs/"
    abs_file_path = os.path.join(script_dir, rel_path)
    if not os.path.exists(abs_file_path):
        os.mkdir(abs_file_path)

    write(abs_file_path + "laser_log.txt", "hello there!")


def write(path, msg):
    with open(path, 'a') as myFile:
        myFile.write(msg)














if __name__ == '__main__':
    main()