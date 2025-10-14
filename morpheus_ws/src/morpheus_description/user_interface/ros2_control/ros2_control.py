import xml.etree.ElementTree as ET
import os

class ROS2Control:

    def __init__(self, ui, path, robot_description):

        self.ui = ui
        self.package_path = path
        self.robot_description = robot_description
        self.root_robot_description = robot_description.root
        self.xacro_tag = '{http://www.ros.org/wiki/xacro}'
        self.mimic = "0"

        self.parse_ros2_control()
        self.parse_ros2_control_template()

        self.comboBox_initialization()
        self.event_initialization()

        self.ressource_changed()

    def parse_ros2_control(self):
        self.ros2_control_path = os.path.join(self.package_path, 'urdf/03_ros2_control/ros2_control.xacro')
        self.tree_ros2control = ET.parse(self.ros2_control_path)
        self.root_ros2control = self.tree_ros2control.getroot()

    def parse_ros2_control_template(self):
        self.ros2_control_template_path = os.path.join(self.package_path, 'urdf/04_templates/ros2_control.template.xacro') 
        self.tree_macro = ET.parse(self.ros2_control_template_path)
        self.root_macro = self.tree_macro.getroot()
    
    def set_xacro(self, root, current_joint, root_rd):

        if self.ui.radioButton_position_existing.isChecked():
            command_interface = 'position'
        elif self.ui.radioButton_velocity_existing.isChecked():
            command_interface = 'velocity'
        elif self.ui.radioButton_effort_existing.isChecked():
            command_interface = 'effort'

        if current_joint == 'dummy':
            joint_name = str(self.ui.comboBox_ros2control_missing.currentText() + '_joint')
            for self.element_rd in root_rd.iter('{http://www.ros.org/wiki/xacro}' + 'part'):
                if self.element_rd.attrib.get('name') == self.ui.comboBox_ros2control_missing.currentText():
                    # nach joint_name in xacro suchen
                    # mimic attr = 1? -> multiplier, offset, joint
                    self.mimic = self.element_rd.attrib.get('mimic')
                    self.joint_mimic = self.element_rd.find('{http://www.ros.org/wiki/xacro}' + 'joint_mimic')
                    self.joint_multiplier = self.joint_mimic.get('multiplier')
                    self.joint_offset = self.joint_mimic.get('offset')
                    self.joint_mimic_joint = self.joint_mimic.get('joint')
        else: 
            joint_name = str(self.ui.comboBox_ros2control_existing.currentText() + '_joint')

        for self.element in root.iter('joint'):
            if self.element.attrib.get('name') == str(current_joint + '_joint'):

                self.element.set('name', joint_name)
                command_interface_tag = self.element.find('command_interface')
                command_interface_tag.set('name', command_interface)

                if self.mimic == '1':
                    mulptiplier = ET.SubElement(self.element, 'param', {'name':'multiplier'})
                    mulptiplier.text = self.joint_multiplier
                    offset = ET.SubElement(self.element, 'param', {'name':'offset'})
                    offset.text = self.joint_offset
                    mimic_joint = ET.SubElement(self.element, 'param', {'name':'mimic'})
                    mimic_joint.text = self.joint_mimic_joint + '_joint'

                return self.element

    def read_xacro(self, root, current_joint):

        self.command_interface = None
        for self.element in root.iter('joint'):
            if self.element.attrib.get('name') == str(current_joint + '_joint'):
                command_interface_tag = self.element.find('command_interface')
                self.command_interface = command_interface_tag.attrib.get('name')

    def set_user_interface(self): 

        if self.command_interface == 'position':
            self.ui.radioButton_position_existing.setChecked(True)
        elif self.command_interface == 'velocity':
            self.ui.radioButton_velocity_existing.setChecked(True)
        elif self.command_interface == 'effort':
            self.ui.radioButton_effort_existing.setChecked(True)

    def add_joint(self):

        if self.ui.comboBox_ros2control_missing.currentText():
            self.parse_ros2_control_template()
            root = self.root_macro
            current_joint = 'dummy'
            element = self.set_xacro(root, current_joint, self.root_robot_description)
            self.root_ros2control[0].append(element)
            self.write_xacro()
            current_joint = self.ui.comboBox_ros2control_missing.currentText()
            index = self.ui.comboBox_ros2control_missing.findText(current_joint)
            self.ui.comboBox_ros2control_missing.removeItem(index)
            self.ui.comboBox_ros2control_existing.addItem(current_joint)

    def remove_joint(self):

        current_joint = self.ui.comboBox_ros2control_existing.currentText()
        for self.element in self.root_ros2control.iter('joint'):
            if self.element.attrib.get('name') == str(current_joint + '_joint'):
                self.root_ros2control[0].remove(self.element)
                self.write_xacro()
                index = self.ui.comboBox_ros2control_existing.findText(current_joint)
                self.ui.comboBox_ros2control_existing.removeItem(index)
                self.ui.comboBox_ros2control_missing.addItem(current_joint)

    def event_initialization(self):

        self.ui.ros2_modify_button.clicked.connect(self.set_ressource_manager_via_ui)
        self.ui.add_button_ros2.clicked.connect(self.add_joint)
        self.ui.ros2_remove_button.clicked.connect(self.remove_joint)

        self.ui.comboBox_ros2control_existing.currentTextChanged.connect(self.ressource_changed)
    
    def comboBox_initialization(self):

        # self.get_missing_interface:
        interfaces = self.get_interfaces()
        moveable_links = self.get_links_with_moveable_joints()
        set1 = set(moveable_links)
        set2 = set(interfaces)
        missing = list(sorted(set1-set2))
        self.ui.comboBox_ros2control_existing.clear()
        self.ui.comboBox_ros2control_existing.addItems(interfaces)
        self.ui.comboBox_ros2control_missing.clear()
        self.ui.comboBox_ros2control_missing.addItems(missing)

    def set_ressource_manager_via_ui(self):
        if self.robot_description.robot_description_xacro.name_changed == True:
            current_joint = self.robot_description.robot_description_xacro.old_name
        else:
            current_joint = self.ui.comboBox_ros2control_existing.currentText()

        root = self.root_ros2control
        self.set_xacro(root, current_joint, self.root_robot_description)
        self.write_xacro()

    def ressource_changed(self):
        root = self.root_ros2control
        current_joint = self.ui.comboBox_ros2control_existing.currentText()

        if self.robot_description.robot_description_xacro.name_changed == True:

            self.read_xacro(root, self.robot_description.robot_description_xacro.old_name)
            self.set_user_interface()
            self.set_ressource_manager_via_ui()
        else:
            current_joint = self.ui.comboBox_ros2control_existing.currentText()
            self.read_xacro(root, current_joint)
            self.set_user_interface()

    def write_xacro(self):
        ET.register_namespace("xacro", 'http://www.ros.org/wiki/xacro')
        ET.indent(self.tree_ros2control,'  ', level=0)
        self.tree_ros2control.write(self.ros2_control_path)

    def get_missing_interfaces(self):

        interfaces = self.get_interfaces()
        moveable_links = self.get_links_with_moveable_joints()
        set1 = set(moveable_links)
        set2 = set(interfaces)
        missing = list(sorted(set1-set2))
        self.ui.comboBox_ros2control_existing.clear()
        self.ui.comboBox_ros2control_existing.addItems(interfaces)
        self.ui.comboBox_ros2control_missing.clear()
        self.ui.comboBox_ros2control_missing.addItems(missing)

    def get_interfaces(self):
        joint_list = []
        for self.element in self.root_ros2control.iter('joint'):
            joint_name = self.element.attrib.get('name')
            joint_name = joint_name.replace('_joint','')
            joint_list.append(joint_name)
        return joint_list
    
    def get_links_with_moveable_joints(self):

        self.moveable_links_list = []
        for self.element in self.root_robot_description.iter(self.xacro_tag + 'part'):
            if self.element.attrib.get('type') != 'fixed':      
                self.moveable_links_list.append(self.element.attrib.get('name'))
        
        return self.moveable_links_list