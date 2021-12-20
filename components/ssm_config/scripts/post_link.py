import sys
import os
from build_util import *
    
def post_link_generate_dfu_package(post_link_target_path, bootloader_hex):
    info = SesPostLink(post_link_target_path)
    info.show()
    
    ret = generate_dfu_package()
    print ret
    


if __name__ == "__main__":
    print sys.path[0], __file__, sys.argv
    post_link_target_path, bootloader_hex = sys.argv[1:]
    
    info = SesPostLink(post_link_target_path)
    info.show()
    
    app_build_info_filename = get_project_build_info_filename(info.project_dir)
    app_timestamp, app_version_string = read_build_info_header(app_build_info_filename)
    app_hex = os.path.splitext(post_link_target_path)[0] + '.hex'
    print app_timestamp, app_version_string
    
    bootloader_project_dir = get_project_dir_from_output_path(bootloader_hex)
    bootloader_build_info_filename = get_project_build_info_filename(bootloader_project_dir)
    bootloader_timestamp, bootloader_version_string = read_build_info_header(bootloader_build_info_filename)
    priv_key = os.path.join(bootloader_project_dir, "resource", "private.pem")
    
    setting_page_file = os.path.join(os.path.dirname(post_link_target_path), "setting_page.hex")
    dfu_zip_file = os.path.join(os.path.dirname(post_link_target_path), "{n}.zip".format(n=app_version_string))
    
    ret = generate_setting_page(app_hex, app_timestamp, bootloader_timestamp, setting_page_file)
    print ret
    
    ret = generate_dfu_package(app_hex, app_timestamp, SD_REQ_S132_6_0_0, priv_key, dfu_zip_file, is_debug=True)
    print ret
    