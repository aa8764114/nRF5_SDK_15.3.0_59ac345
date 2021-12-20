from shutil import copy
import os
import sys
import subprocess
import datetime
import re

BUILD_INFO_HEADER_TEMPLATE = '#ifndef BUILD_INFO_H__\n#define BUILD_INFO_H__\n\n#define BUILD_TIME {t}\n#define GIT_VERSION "{v}"\n\n#endif     // BUILD_INFO_H__\n'

SD_REQ_S112_6_0_0   = 0xA7
SD_REQ_S112_6_1_0   = 0xB0
SD_REQ_S132_6_0_0   = 0xA8
SD_REQ_S132_6_1_0   = 0xAF

def get_project_build_info_filename(project_dir):
    return os.path.join(project_dir, "build_info.h")

def get_build_info_header_content(timestamp, version_string):
    return BUILD_INFO_HEADER_TEMPLATE.format(t=timestamp, v=version_string)

def is_git_clean():
    return subprocess.check_output("git status -s".split(' ')).strip() == ''
    
def exec_cmd(command):
    print command
    cmd_list = command.split(' ')
    if "|" in cmd_list:
#    	print "using Popen"
        # commands with pipe, generally more complicated ones, may only be executed through this more dangerous method
#        return subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT).communicate()[0]
        return subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=None).communicate()[0]
    else:
#    	print "using check_output"
        # A safer usage that should be used in most cases
        return subprocess.check_output(cmd_list, shell=False)

def get_timestamp():
    def get_timestamp_HEAD():
        ret = exec_cmd('git log -1 --date=unix | grep "Date:"')
        return int(ret.split(' ')[-1])
    def get_timestamp_last_modified():
        ret = exec_cmd('git status -s | while read mode file; do echo $(stat -f "%m" $file) $mode $file; done | sort -r')
        for line in ret.splitlines():
        	try:
        		return int(line.split(' ')[0])
        	except:
        		pass
#         return int(ret.split(' ')[0])
    return get_timestamp_HEAD() if is_git_clean() else get_timestamp_last_modified()

def get_version_string(timestamp):
#    return "9fe262-dirty-2018-04-24-12:42:12"
    timestamp_string = datetime.datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d-%H:%M:%S")
    ret = exec_cmd("git describe --abbrev=6 --dirty=-dirty-{} --always --tag".format(timestamp_string))
#    ret = exec_cmd("git describe --abbrev=6 --dirty=-dirty-2018-04-24-12:42:12 --always")
#    ret = exec_cmd("echo 9fe262-dirty-2018-04-24-12:42:12")
    print ret
    return ret.rstrip()

def get_project_dir_from_output_path(output_path):
    return os.path.dirname(output_path[:output_path.find("Output/")])

class SesPostLink:
    def __init__(self, target_path):
        self.out_dir = os.path.dirname(target_path)
        self.project_name = os.path.splitext(os.path.split(target_path)[1])[0]
        self.configuration = "Release" if "/Output/Release/" in target_path else "Debug"
        self.project_dir = get_project_dir_from_output_path(target_path)
    def show(self):
        print "BuildInfo:"
        print '\n'.join("%s: %s" % item for item in vars(self).items())
        
def read_build_info_header(filename):
    re_template = BUILD_INFO_HEADER_TEMPLATE.replace('{t}', '(\d+)').replace('{v}', '(\S+)')
    data = open(filename, 'rb').read()
    m = re.match(re_template, data)
    return m.groups()
            

def generate_setting_page(app_hexname, app_ver, bootloader_ver, bootloader_setting_hex):
    command_base = "/usr/local/bin/nrfutil settings generate --family NRF52 --application {s} --application-version {a} --bootloader-version {b} --bl-settings-version 1 --no-backup {o}"
    return exec_cmd(command_base.format(s=app_hexname, a=app_ver, b=bootloader_ver, o=bootloader_setting_hex))


def generate_dfu_package(app_hexname, app_ver, sd_req, priv_key, zip_out, is_debug=False):
    command_base = "/usr/local/bin/nrfutil pkg generate{d} --application {a} --application-version {v} --hw-version 52 --sd-req {r} --key-file {k} {o}"
    debug_flag = " --debug-mode" if is_debug else ""
    return exec_cmd(command_base.format(d=debug_flag, a=app_hexname, v=app_ver, r=sd_req, k=priv_key, o=zip_out))





'''

class BuildInfo:
    def __init__(self, target_path):
        self.out_dir = os.path.dirname(target_path)
        self.proj_name = os.path.splitext(os.path.split(target_path)[1])[0]
        self.configuration = "Release" if "/Output/Release/" in target_path else "Debug"
        self.timestamp = get_timestamp()
        self.version_string = get_version_string(self.timestamp)
    def show(self):
        print "BuildInfo:"
        print '\n'.join("%s: %s" % item for item in vars(self).items())
    def ls(self):
        ret = exec_cmd("ls -la {d}".format(d=self.out_dir))
        print ret
    def gen_header_data(self, timestamp, version_string):
        template = '#ifndef BUILD_INFO_H__\n#define BUILD_INFO_H__\n\n#define BUILD_TIME ({t})\n#define GIT_VERSION ("{v}")\n\n#endif     // BUILD_INFO_H__\n'
        return template.format(t=timestamp, v=version_string)
    
    def to_header(self, filename):
        header_template = '#ifndef BUILD_INFO_H__\n#define BUILD_INFO_H__\n\n#define BUILD_TIME ({t})\n#define GIT_VERSION ("{v}")\n\n#endif     // BUILD_INFO_H__\n'
        with open(filename, 'wb') as f:
            f.write(header_content.format(self.timestamp, self.version_string))
            
            
def get_build_time_from_build_info_file(build_info_file):
    def get_feature(_data, feature_line_start, feature_line_end):
        assert _data.count(feature_line_start) == 1
        idx_start = _data.find(feature_line_start)
        idx_end = _data.find(feature_line_end, idx_start)
        return _data[idx_start+len(feature_line_start):idx_end]
    with open(build_info_file, 'rb') as f:
        data = f.read()
    int_ver = int(get_feature(data, "#define BUILD_TIME (", ")"))
    str_ver = get_feature(data, '#define GIT_VERSION ("', '")').replace(':', '-')
    return int_ver, str_ver

def generate_dfu_setting(src, app_build_info_file, bld_build_info_file, bld_setting_hex):
    app_ver, app_ver_string = get_build_time_from_build_info_file(app_build_info_file)
    bld_ver, bld_ver_string = get_build_time_from_build_info_file(bld_build_info_file)
    command = "/usr/local/bin/nrfutil settings generate --family NRF52 --application {s} --application-version {a} --bootloader-version {b} --bl-settings-version 1 {o}".format(s=src, a=app_ver, b = bld_ver, o=bld_setting_hex)
    print command
    ret = subprocess.check_output(command.split(' ')).strip()

def generate_dfu_package(app_hex, app_build_info_h, sd_req, priv_key, configuration):
    """
    SD, APP, BL should DFU separately since SD is the largest section (140 kB for s132 5.1.0) and nrfutil does not support BL + APP (use two packages instead).
    We only need to package APP-only zip in post-build script for APP project
    """
    hex_dir = os.path.dirname(app_hex)
    app_ver, app_ver_string = get_build_time_from_build_info_file(app_build_info_file)
    zip_out = os.path.join(hex_dir, "fuhsing_sesame1.1-{s}.zip".format(s=app_ver_string))
    
    command = "rm {n}".format(n=os.path.join(hex_dir, "fuhsing_sesame1.1-*-dirty-*.zip"))
    print command
    os.system(command)
    
    debug_flag = " --debug-mode" if configuration == "Debug" else ""
    command_format = "/usr/local/bin/nrfutil pkg generate{d} --application {a} --application-version {v} --hw-version 52 --sd-req {r} --key-file {k} {o}"
    command = command_format.format(d=debug_flag, a=app_hex, v=app_ver, r=sd_req, k=priv_key, o=zip_out)
    print command
    try:
        ret = subprocess.check_output(command.split(' ')).strip()
        return zip_out
    except:
        raise
'''