import sys
from build_util import *

def update_project_build_info(project_dir):
    timestamp = get_timestamp()
    version_string = get_version_string(timestamp)
    expected_data = get_build_info_header_content(timestamp, version_string)
    
    project_build_info_filename = get_project_build_info_filename(project_dir)
    if os.path.isfile(project_build_info_filename) and open(project_build_info_filename, 'rb').read() == expected_data:
        print "{n} is up to date".format(n=project_build_info_filename)
    else:
        with open(project_build_info_filename, 'wb') as f:
            f.write(expected_data)
        print "{n} is updated: {t}, {v}".format(n=project_build_info_filename, t=timestamp, v=version_string)
        

if __name__ == "__main__":
    print sys.path[0], __file__, sys.argv
    
    project_dir = sys.argv[1]
    update_project_build_info(project_dir)
    