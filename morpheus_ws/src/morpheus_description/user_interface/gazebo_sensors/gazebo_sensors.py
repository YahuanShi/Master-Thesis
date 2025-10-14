import os
import xml.etree.ElementTree as ET
from   PyQt6.QtWidgets import QMessageBox

from gazebo_sensors.packages.camera import GazeboCamera
from gazebo_sensors.packages.lidar import GazeboLidar
from gazebo_sensors.packages.imu import GazeboIMU
from gazebo_sensors.packages.force_torque import GazeboForceTorque
from gazebo_sensors.packages.functions import GazeboFunctions

class GazeboSensors:

    def __init__(self, ui, path, robot_description):

        self.ui = ui
        self.package_path = path
        self.robot_description = robot_description

        self.parse_gazebo_sensors()
        self.parse_gazebo_sensors_template()

        self.camera = GazeboCamera(self.ui, self.root_gazebo, self.package_path)
        self.lidar = GazeboLidar(self.ui, self.root_gazebo, self.package_path)
        self.imu = GazeboIMU(self.ui, self.root_gazebo, self.package_path)
        self.forcetorque = GazeboForceTorque(self.ui, self.root_gazebo, self.package_path)
        self.gazebo_functions = GazeboFunctions(self.gazebo_sensors_path)

        self.comboBox_initialization()
        self.event_initialization()

        self.set_gui_camera()
        self.set_gui_lidar()
        self.set_gui_imu()
        self.set_gui_ft()
            
    def parse_gazebo_sensors(self):
        self.gazebo_sensors_path = os.path.join(self.package_path, 'urdf/02_gazebo/gazebo.xacro')
        self.tree_gazebo = ET.parse(self.gazebo_sensors_path)
        self.root_gazebo = self.tree_gazebo.getroot()

    def parse_gazebo_sensors_template(self):
        self.gazebo_sensors_template_path = os.path.join(self.package_path, 'urdf/04_templates/gazebo.template.xacro') 
        self.tree_sensors = ET.parse(self.gazebo_sensors_template_path)
        self.root_sensors = self.tree_sensors.getroot()

    """
    --------------------------------------------------------------------------------------------------------
    -------------------------------------Xacro related Functions--------------------------------------------
    --------------------------------------------------------------------------------------------------------
    """
    
    def set_gui_camera(self):

        if self.ui.camera_select.currentText():

            self.camera.read_camera()
            self.camera.set_user_interface()

            if self.robot_description.robot_description_xacro.name_changed == True and self.ui.camera_select.count() > 1 and self.camera.camera_name != "new camera":
                index = self.ui.camera_reference.findText(self.camera.camera_reference.replace('_link',''))
                self.ui.camera_reference.insertItem(index+1, self.robot_description.robot_description_xacro.name)
                self.ui.camera_reference.removeItem(index)
                if self.camera.camera_reference.replace('_link','') != self.ui.camera_reference.currentText():
                    self.modify_camera()
                    self.camera.read_camera()

    def set_gui_lidar(self):

        if self.ui.lidar_select.currentText():
            self.lidar.read_lidar()
            self.lidar.set_user_interface()

            if self.robot_description.robot_description_xacro.name_changed == True and self.ui.lidar_select.count() > 1 and self.lidar.lidar_name != "new lidar":
                index = self.ui.lidar_reference.findText(self.lidar.lidar_reference.replace('_link',''))
                self.ui.lidar_reference.insertItem(index+1, self.robot_description.robot_description_xacro.name)
                self.ui.lidar_reference.removeItem(index)
                if self.lidar.lidar_reference.replace('_link','') != self.ui.lidar_reference.currentText():
                    self.modify_lidar()
                    self.lidar.read_lidar()

    def set_gui_imu(self):
        if self.ui.imu_select.currentText():
            self.imu.read_imu()
            self.imu.set_user_interface()

            if self.robot_description.robot_description_xacro.name_changed == True and self.ui.imu_select.count() > 1 and self.imu.imu_name != "new imu":
                index = self.ui.imu_reference.findText(self.imu.imu_reference.replace('_link',''))
                self.ui.imu_reference.insertItem(index+1, self.robot_description.robot_description_xacro.name)
                self.ui.imu_reference.removeItem(index)
                if self.imu.imu_reference.replace('_link','') != self.ui.imu_reference.currentText():
                    self.modify_imu()
                    self.imu.read_imu()

    def set_gui_ft(self):
        if self.ui.ft_select.currentText():
            self.forcetorque.read_force_torque()
            self.forcetorque.set_user_interface()

            if self.robot_description.robot_description_xacro.name_changed == True and self.ui.ft_select.count() > 1 and self.forcetorque.ft_name != "new ft":
                index = self.ui.ft_reference.findText(self.forcetorque.ft_reference.replace('_joint',''))
                self.ui.ft_reference.insertItem(index+1, self.robot_description.robot_description_xacro.name)
                self.ui.ft_reference.removeItem(index)
                if self.forcetorque.ft_reference.replace('_joint','') != self.ui.ft_reference.currentText():
                    self.modify_ft()
                    self.forcetorque.read_force_torque()

    def modify_camera(self):

        all_camera = [self.ui.camera_select.itemText(i) for i in range(self.ui.camera_select.count())]
        new_camera = self.ui.camera_name.text() + " | " + self.ui.camera_reference.currentText() + '_link'
        current_camera = self.camera.camera_name + " | " + self.camera.camera_reference

        if  (new_camera in all_camera and new_camera != current_camera) or self.ui.camera_name.text() == 'new camera':
            self.dupes_warning()
        else:
            name = self.camera.camera_name
            reference = self.camera.camera_reference
            camera_select = str(name + ' | ' + reference)

            new_name = self.ui.camera_name.text()
            new_reference = self.ui.camera_reference.currentText()
            new_camera = str(new_name + ' | ' + new_reference + '_link')

            root = self.tree_gazebo.getroot() #????
            self.camera.set_camera(root)
            self.gazebo_functions.write_file(self.tree_gazebo)

            index = self.ui.camera_select.findText(camera_select)
            self.ui.camera_select.setItemText(index, new_camera)

    def modify_lidar(self):

        all_lidar = [self.ui.lidar_select.itemText(i) for i in range(self.ui.lidar_select.count())]
        new_lidar = self.ui.lidar_name.text() + " | " + self.ui.lidar_reference.currentText() + '_link'
        current_lidar = self.lidar.lidar_name + " | " + self.lidar.lidar_reference

        if (new_lidar in all_lidar and new_lidar != current_lidar) or self.ui.lidar_name.text() == 'new lidar':
            self.dupes_warning()
        else:
            name = self.lidar.lidar_name
            reference = self.lidar.lidar_reference
            lidar_select = str(name + ' | ' + reference)

            new_name = self.ui.lidar_name.text()
            new_reference = self.ui.lidar_reference.currentText()
            new_lidar = str(new_name + ' | ' + new_reference + '_link')

            root = self.tree_gazebo.getroot()
            self.lidar.set_lidar(root)
            self.gazebo_functions.write_file(self.tree_gazebo)

            index = self.ui.lidar_select.findText(lidar_select)
            self.ui.lidar_select.setItemText(index, new_lidar)
    
    def modify_imu(self):

        all_imu = [self.ui.imu_select.itemText(i) for i in range(self.ui.imu_select.count())]
        new_imu = self.ui.imu_name.text() + " | " + self.ui.imu_reference.currentText() + '_link'
        current_imu = self.imu.imu_name + " | " + self.imu.imu_reference

        if (new_imu in all_imu and new_imu != current_imu) or self.ui.imu_name.text() == 'new imu':
            self.dupes_warning()
        else:

            name = self.imu.imu_name
            reference = self.imu.imu_reference
            imu_select = str(name + ' | ' + reference)

            new_name = self.ui.imu_name.text()
            new_reference = self.ui.imu_reference.currentText()
            new_imu = str(new_name + ' | ' + new_reference + '_link')
            
            root = self.tree_gazebo.getroot() #???
            self.imu.set_imu(root)
            self.gazebo_functions.write_file(self.tree_gazebo)

            index = self.ui.imu_select.findText(imu_select)
            self.ui.imu_select.setItemText(index, new_imu)

    def modify_ft(self):

        all_ft = [self.ui.ft_select.itemText(i) for i in range(self.ui.ft_select.count())]
        new_ft = self.ui.ft_name.text() + " | " + self.ui.ft_reference.currentText() + '_joint'
        current_ft = self.forcetorque.ft_name + " | " + self.forcetorque.ft_reference
        if (new_ft in all_ft and new_ft != current_ft) or self.ui.ft_name.text() == 'new ft':
            self.dupes_warning()
        else:        
            name = self.forcetorque.ft_name
            reference = self.forcetorque.ft_reference
            ft_select = str(name + ' | ' + reference)

            new_name = self.ui.ft_name.text()
            new_reference = self.ui.ft_reference.currentText()
            new_ft = str(new_name + ' | ' + new_reference + '_joint')

            root = self.tree_gazebo.getroot() #???
            self.forcetorque.set_ft(root)
            self.gazebo_functions.write_file(self.tree_gazebo)

            index = self.ui.ft_select.findText(ft_select)
            self.ui.ft_select.setItemText(index, new_ft)


    def add_camera(self):

        all_camera = [self.ui.camera_select.itemText(i) for i in range(self.ui.camera_select.count())]
        camera = self.ui.camera_name.text() + " | " + self.ui.camera_reference.currentText() + '_link'

        if  camera in all_camera or self.ui.camera_name.text() == 'new camera':
            self.dupes_warning()
        else:
            self.parse_gazebo_sensors_template()
            element = self.camera.set_camera(self.root_sensors)
            self.root_gazebo.append(element)
            self.gazebo_functions.write_file(self.tree_gazebo)

            index = self.ui.camera_select.count() - 1
            new_name = self.ui.camera_name.text()
            new_reference = self.ui.camera_reference.currentText()
            new_select = str(new_name + ' | ' + new_reference + '_link')

            self.ui.camera_select.insertItem(index, new_select)
            self.ui.camera_select.setCurrentText(new_select)

    def add_lidar(self):

        all_lidar = [self.ui.lidar_select.itemText(i) for i in range(self.ui.lidar_select.count())]
        lidar = self.ui.lidar_name.text() + " | " + self.ui.lidar_reference.currentText() + '_link'
        if lidar in all_lidar or self.ui.lidar_name.text() == 'new lidar':
            self.dupes_warning()
        else:
            self.parse_gazebo_sensors_template()
            element = self.lidar.set_lidar(self.root_sensors)
            self.root_gazebo.append(element)
            self.gazebo_functions.write_file(self.tree_gazebo)

            index = self.ui.lidar_select.count() - 1
            new_name = self.ui.lidar_name.text()
            new_reference = self.ui.lidar_reference.currentText()
            new_select = str(new_name + ' | ' + new_reference + '_link')
            self.ui.lidar_select.insertItem(index, new_select)
            self.ui.lidar_select.setCurrentText(new_select)

    def add_imu(self):

        all_imu = [self.ui.imu_select.itemText(i) for i in range(self.ui.imu_select.count())]
        imu = self.ui.imu_name.text() + " | " + self.ui.imu_reference.currentText() + '_link'
        if imu in all_imu or self.ui.imu_name.text() == 'new imu':
            self.dupes_warning()
        else:
            self.parse_gazebo_sensors_template()
            element = self.imu.set_imu(self.root_sensors)
            self.root_gazebo.append(element)
            self.gazebo_functions.write_file(self.tree_gazebo)

            index = self.ui.imu_select.count() - 1
            new_name = self.ui.imu_name.text()
            new_reference = self.ui.imu_reference.currentText()
            new_select = str(new_name + ' | ' + new_reference + '_link')
            self.ui.imu_select.insertItem(index, new_select)
            self.ui.imu_select.setCurrentText(new_select)

    def add_ft(self):

        all_ft = [self.ui.ft_select.itemText(i) for i in range(self.ui.ft_select.count())]
        ft = self.ui.ft_name.text() + " | " + self.ui.ft_reference.currentText() + '_joint'
        if ft in all_ft or self.ui.ft_name.text() == 'new ft':
            self.dupes_warning()
        else:
            self.parse_gazebo_sensors_template()
            element = self.forcetorque.set_ft(self.root_sensors)
            self.root_gazebo.append(element)
            self.gazebo_functions.write_file(self.tree_gazebo)

            index = self.ui.ft_select.count() - 1
            new_name = self.ui.ft_name.text()
            new_reference = self.ui.ft_reference.currentText()
            new_select = str(new_name + ' | ' + new_reference + '_joint')
            self.ui.ft_select.insertItem(index, new_select)
            self.ui.ft_select.setCurrentText(new_select)

    def remove_camera(self):

        camera_select = self.ui.camera_select.currentText()
        camera_split = camera_select.split()
        camera_name = camera_split[0]
        camera_reference = camera_split[2]

        self.gazebo_functions.remove_sensor(self.root_gazebo, camera_name, camera_reference)
        self.gazebo_functions.write_file(self.tree_gazebo)

        index = self.ui.camera_select.findText(camera_select)  # find the index of text
        self.ui.camera_select.removeItem(index)

    def remove_lidar(self):

        lidar_select = self.ui.lidar_select.currentText()
        lidar_split = lidar_select.split()
        lidar_name = lidar_split[0]
        lidar_reference = lidar_split[2]

        self.gazebo_functions.remove_sensor(self.root_gazebo, lidar_name, lidar_reference)
        self.gazebo_functions.write_file(self.tree_gazebo)

        index = self.ui.lidar_select.findText(lidar_select)  # find the index of text
        self.ui.lidar_select.removeItem(index)

    def remove_imu(self):

        imu_select = self.ui.imu_select.currentText()
        imu_split = imu_select.split()
        imu_name = imu_split[0]
        imu_reference = imu_split[2]

        self.gazebo_functions.remove_sensor(self.root_gazebo, imu_name, imu_reference)
        self.gazebo_functions.write_file(self.tree_gazebo)

        index = self.ui.imu_select.findText(imu_select)
        self.ui.imu_select.removeItem(index)

    def remove_ft(self):

        ft_select = self.ui.ft_select.currentText()
        ft_split = ft_select.split()
        ft_name = ft_split[0]
        ft_reference = ft_split[2]

        self.gazebo_functions.remove_sensor(self.root_gazebo, ft_name, ft_reference)
        self.gazebo_functions.write_file(self.tree_gazebo)

        index = self.ui.ft_select.findText(ft_select)  # find the index of text
        self.ui.ft_select.removeItem(index)

    """
    --------------------------------------------------------------------------------------------------------
    -------------------------------------GUI related Functions----------------------------------------------
    --------------------------------------------------------------------------------------------------------
    """

    def event_initialization(self):

        self.ui.camera_modify_button.clicked.connect(self.modify_camera)
        self.ui.camera_add_button.clicked.connect(self.add_camera)
        self.ui.camera_remove_button.clicked.connect(self.remove_camera)
        self.ui.lidar_modify_button.clicked.connect(self.modify_lidar)
        self.ui.lidar_add_button.clicked.connect(self.add_lidar)
        self.ui.lidar_remove_button.clicked.connect(self.remove_lidar)
        self.ui.imu_modify_button.clicked.connect(self.modify_imu)
        self.ui.imu_add_button.clicked.connect(self.add_imu)
        self.ui.imu_remove_button.clicked.connect(self.remove_imu)
        self.ui.ft_modify_button.clicked.connect(self.modify_ft)
        self.ui.ft_add_button.clicked.connect(self.add_ft)
        self.ui.ft_remove_button.clicked.connect(self.remove_ft)
        self.sensor_selection()
        self.ui.camera_radio.toggled.connect(self.sensor_selection)
        self.ui.lidar_radio.toggled.connect(self.sensor_selection)
        self.ui.imu_radio.toggled.connect(self.sensor_selection)
        self.ui.ft_radio.toggled.connect(self.sensor_selection)

        self.ui.camera_select.currentTextChanged.connect(self.set_gui_camera)
        self.ui.lidar_select.currentTextChanged.connect(self.set_gui_lidar)
        self.ui.imu_select.currentTextChanged.connect(self.set_gui_imu)
        self.ui.ft_select.currentTextChanged.connect(self.set_gui_ft)
        self.lidar_orientation()
        self.ui.comboBox_lidar_hv.currentTextChanged.connect(self.lidar_orientation)

    def comboBox_initialization(self):

        links_list = [self.ui.comboBox.itemText(i) for i in range(self.ui.comboBox.count()-1)]
        [a, b, c, d] = self.gazebo_functions.get_sensor_names(self.root_gazebo)
        self.ui.camera_select.clear()
        self.ui.camera_select.addItems(a)
        self.ui.camera_select.addItem('new camera')
        self.ui.camera_reference.clear()
        self.ui.camera_reference.addItems(links_list)

        self.ui.lidar_select.clear()
        self.ui.lidar_select.addItems(b)
        self.ui.lidar_select.addItem('new lidar')
        self.ui.lidar_reference.clear()
        self.ui.lidar_reference.addItems(links_list)

        self.ui.imu_select.clear()
        self.ui.imu_select.addItems(c)
        self.ui.imu_select.addItem('new imu')
        self.ui.imu_reference.clear()
        self.ui.imu_reference.addItems(links_list)

        self.ui.ft_select.clear()
        self.ui.ft_select.addItems(d)
        self.ui.ft_select.addItem('new ft')
        self.ui.ft_reference.clear()
        self.ui.ft_reference.addItems(links_list)

    def sensor_selection(self):

        self.ui.camera_group.setVisible(self.ui.camera_radio.isChecked())
        self.ui.lidar_group.setVisible(self.ui.lidar_radio.isChecked())
        self.ui.imu_group.setVisible(self.ui.imu_radio.isChecked())
        self.ui.ft_group.setVisible(self.ui.ft_radio.isChecked())

    def lidar_orientation(self):
        if self.ui.comboBox_lidar_hv.currentText() == 'Horizontal':
            visualize = True
        else:
            visualize = False

        self.ui.lidar_min_angle_h.setVisible(visualize)
        self.ui.lidar_max_angle_h.setVisible(visualize)
        self.ui.lidar_resolution_h.setVisible(visualize)
        self.ui.lidar_samples_h.setVisible(visualize)
        self.ui.lidar_min_angle_v.setVisible(not visualize)
        self.ui.lidar_max_angle_v.setVisible(not visualize)
        self.ui.lidar_resolution_v.setVisible(not visualize)
        self.ui.lidar_samples_v.setVisible(not visualize)

    def dupes_warning(self):

        msg = QMessageBox()
        msg.setIcon(msg.Icon.Warning)
        msg.setStyleSheet("QLabel{min-height:100 px} QLabel{subcontrol-position: top center}")
        msg.setText("There is a Duplicate! Please choose a valid name for the link first!        ")
        msg.setWindowTitle("Warning MessageBox")
        msg.setStandardButtons(msg.StandardButton.Ok)
        msg.exec()