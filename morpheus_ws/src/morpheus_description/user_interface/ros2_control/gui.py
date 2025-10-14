from PyQt6 import QtWidgets, uic
from PyQt6.QtWidgets import QMainWindow, QFileDialog, QInputDialog, QColorDialog, QMessageBox

import sys, time, os, shutil

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
from datetime import datetime

from robot_description.robot_description import RobotDescription
from gazebo_sensors.gazebo_sensors import GazeboSensors
from ros2_control.ros2_control import ROS2Control
from urdf_import.urdf_importer import URDFImporter
from ros2.ros2_tools import ROS2Tools

class UserInterface(QMainWindow):

    def __init__(self):

        QMainWindow.__init__(self)

        package_name = 'morpheus_description'
        import os, sys
        
        # 1. 获取当前工作目录 (CWD)
        # CWD 预期是: /home/syh/Documents/Master Thesis/morpheus_ws/src/morpheus_description/user_interface
        current_dir = os.getcwd() 
        
        # 2. 从 CWD 向上追溯两级，到达包的根目录
        # current_dir/.. (morpheus_description) /.. (src)
        
        # 强制使用硬编码的相对路径结构来定位包的根
        # 包根目录 = CWD的上上级目录/morpheus_description
        
        # 找到 'src' 目录的路径: current_dir/../..
        src_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
        
        # 构造正确的包根目录路径：src_dir/morpheus_description
        self.absolute_package_path = os.path.join(src_dir, package_name)
        
        # 调试输出
        print(f"DEBUG: Calculated Package Path: {self.absolute_package_path}") 
        sys.stdout.flush()

        # package_prefix = get_package_prefix(package_name)
        # package_prefix = package_prefix.replace('install', 'src')
        # self.absolute_package_path = package_prefix 

        self.user_interface_initialization()
    

    def user_interface_initialization(self):

        self.ui = uic.loadUi('gui.ui', self)
    
        self.morpheus = ROS2Tools(self.absolute_package_path)
        self.robot_description = RobotDescription(self.ui, self.morpheus, self.absolute_package_path)
        self.gazebo_sensors = GazeboSensors(self.ui, self.absolute_package_path, self.robot_description)
        self.ros2_control = ROS2Control(self.ui, self.absolute_package_path, self.robot_description)
        self.urdf_import = URDFImporter(self.ui, self.robot_description, self.morpheus, self.absolute_package_path)
        self.morpheus.start_processes()

        self.event_inizialization()

    def event_inizialization(self):

        self.actionNew.triggered.connect(self.new_config)
        self.actionLoad.triggered.connect(self.load_config)
        self.actionSave.triggered.connect(self.save_config)
        self.actionURDF.triggered.connect(self.import_urdf)

    def new_config(self):

        msg = QMessageBox()
        msg.setIcon(msg.Icon.Warning)
        msg.setStyleSheet("QLabel{min-height:100 px;} QMessageBox{subcontrol-position: top center}")
        msg.setText("Are you sure you want the override the current configuration? All unsaved changes will be deleted permanently!")
        msg.setWindowTitle("Warning MessageBox")
        msg.setStandardButtons(msg.StandardButton.Yes | msg.StandardButton.Cancel)
        retval = msg.exec()

        if retval == msg.StandardButton.Yes:

            robot_description_new_path = os.path.join(self.absolute_package_path, 'urdf/04_templates/robot_description.new.xacro')
            gazebo_sensors_new_path = os.path.join(self.absolute_package_path, 'urdf/04_templates/gazebo.new.xacro')
            ros2_control_new_path = os.path.join(self.absolute_package_path, 'urdf/04_templates/ros2_control.new.xacro')
            self.parse_and_write_xml(robot_description_new_path, gazebo_sensors_new_path, ros2_control_new_path)

            self.morpheus.stop_processes()
            time.sleep(0.5)
            self.user_interface_initialization()      

    def load_config(self):

        directory = QFileDialog.getExistingDirectory(
            self,
            caption='Select a configuration',
            directory= os.path.join(self.absolute_package_path, 'urdf', '05_save_files')
        )

        load_rd_path = os.path.join(directory, 'robot_description.xacro')
        load_gazebo_path = os.path.join(directory, 'gazebo.xacro')
        load_ros2control_path = os.path.join(directory, 'ros2_control.xacro')
        self.parse_and_write_xml(load_rd_path, load_gazebo_path, load_ros2control_path)

        self.morpheus.stop_processes()
        time.sleep(0.5)
        self.user_interface_initialization()

    def import_urdf(self):

        new_rd_path = os.path.join(self.absolute_package_path, 'urdf/04_templates/robot_description.new.xacro')
        new_gazebo_path = os.path.join(self.absolute_package_path, 'urdf/04_templates/gazebo.new.xacro')
        new_ros2control_path = os.path.join(self.absolute_package_path, 'urdf/04_templates/ros2_control.new.xacro')
        
        self.parse_and_write_xml(new_rd_path, new_gazebo_path, new_ros2control_path)

        self.robot_description.parse_robot_description()
        self.urdf_import.parse_urdf()

        urdf_link_names = self.urdf_import.urdf_import_func.get_urdf_link_names(self.urdf_import.root_urdf)
        
        for link in urdf_link_names:

            self.robot_description.parse_robot_description_template()
            self.urdf_import.urdf_import_gui.read_urdf(link, self.urdf_import.root_urdf)
            self.robot_description.robot_description_xacro.name = link.replace('_link','') # TODO: lassen oder dem Nutzer selbst überlassen? | name von xacro oder urdf?
 
            if urdf_link_names.index(link) == 0:
                self.macro = 'base'
                name_attr = 'base'
                root = self.robot_description.root
                self.robot_description.robot_description_xacro.set_xacro(root, name_attr, self.macro)
            else:
                self.macro = 'part'
                name_attr = 'new link'
                root = self.robot_description.root_template
                self.robot_description.robot_description_xacro.set_xacro(root, name_attr, self.macro)
                self.robot_description.root.append(self.robot_description.robot_description_xacro.element)

        self.robot_description.robot_description_func.write_robot_description(self.robot_description.tree)
        self.morpheus.stop_processes()
        time.sleep(0.5)
        self.user_interface_initialization()  

    def save_config(self):

        time_today = datetime.today().strftime('%y%m%d %H%M')
        datum = time_today.split(' ')[0]
        uhrzeit = time_today.split(' ')[1]

        dialog = QInputDialog()
        dialog.setLabelText('Enter the configuration name:')
        dialog.setTextValue(str('config_' + datum + '_' + uhrzeit))
        dialog.exec()
        
        main_directory= os.path.join(self.absolute_package_path, 'urdf', '05_save_files')
        config_name = dialog.textValue()
        config = os.path.join(main_directory, config_name)

        if config_name in os.listdir(main_directory):
            msg = QMessageBox()
            msg.setIcon(msg.Icon.Warning)
            msg.setStyleSheet("QLabel{min-height:100 px;} QMessageBox{subcontrol-position: top center}")
            msg.setText("Are you sure you want to override an existing configuration?")
            msg.setWindowTitle("Warning MessageBox")
            msg.setStandardButtons(msg.StandardButton.Yes | msg.StandardButton.Cancel)
            retval = msg.exec()

            if retval == msg.StandardButton.Yes:            
                if not os.path.exists(config):
                    os.mkdir(config)
                rd = os.path.join(config,'robot_description.xacro')
                gazebo = os.path.join(config,'gazebo.xacro')
                ros2 = os.path.join(config,'ros2_control.xacro')
                shutil.copyfile(os.path.join(self.absolute_package_path, 'urdf/01_robot_description/robot_description.xacro'), rd)
                shutil.copyfile(os.path.join(self.absolute_package_path, 'urdf/02_gazebo/gazebo.xacro'), gazebo)
                shutil.copyfile(os.path.join(self.absolute_package_path, 'urdf/03_ros2_control/ros2_control.xacro'), ros2)
            else:
                pass
        else:
            if not os.path.exists(config):
                os.mkdir(config)
            rd = os.path.join(config,'robot_description.xacro')
            gazebo = os.path.join(config,'gazebo.xacro')
            ros2 = os.path.join(config,'ros2_control.xacro')
            shutil.copyfile(os.path.join(self.absolute_package_path, 'urdf/01_robot_description/robot_description.xacro'), rd)
            shutil.copyfile(os.path.join(self.absolute_package_path, 'urdf/02_gazebo/gazebo.xacro'), gazebo)
            shutil.copyfile(os.path.join(self.absolute_package_path, 'urdf/03_ros2_control/ros2_control.xacro'), ros2)
            
    def parse_and_write_xml(self, path_rd, path_gz, path_ros2):

        tree_rd = ET.parse(path_rd)
        tree_gz = ET.parse(path_gz)
        tree_ros2 = ET.parse(path_ros2)
        ET.register_namespace("xacro", 'http://www.ros.org/wiki/xacro')
        ET.indent(tree_rd,'  ', level=0)
        ET.indent(tree_gz,'  ', level=0)
        ET.indent(tree_ros2,'  ', level=0)
        tree_rd.write(self.robot_description.robot_description_path)
        tree_gz.write(self.gazebo_sensors.gazebo_sensors_path)
        tree_ros2.write(self.ros2_control.ros2_control_path)   
        
    def closeEvent(self, event):
        self.morpheus.stop_processes()
        self.urdf_import.stop_visualizer()
        event.accept()

if __name__ == '__main__':

    app = QtWidgets.QApplication(sys.argv)
    mainWindow = UserInterface()
    mainWindow.show()
    sys.exit(app.exec())

