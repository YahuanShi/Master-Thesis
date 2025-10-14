import time, tempfile
import xml.etree.ElementTree as ET
import numpy as np
import subprocess, os, signal

from urdf_import.packages.functions import URDFImporterFunctions
from urdf_import.packages.user_interface import URDFImporterGUI

class URDFImporter:

    def __init__(self, ui, robot_description, morpheus, path):

        self.ui = ui
        self.robot_description = robot_description
        self.morpheus = morpheus
        self.package_path = path

        self.robot_description_xacro = self.robot_description.robot_description_xacro

        self.urdf_import_func = URDFImporterFunctions(self.ui)
        self.urdf_import_gui = URDFImporterGUI(self.ui, self.robot_description)

        self.comboBox_initialization()
        self.event_initialization()

        self.import_visualization = False

    """
    --------------------------------------------------------------------------------------------------------
    -------------------------------------Xacro related Functions--------------------------------------------
    --------------------------------------------------------------------------------------------------------
    """

    def parse_urdf(self):

        self.urdf_file = self.urdf_import_func.select_urdf_file() 
        self.path_pkg = self.urdf_import_func.select_urdf_package() 

        print(self.path_pkg)

        if self.urdf_file != '' and self.path_pkg != '':
            pkg_name = self.path_pkg.split('/')[-1]
            # Read and modify the URDF content
            with open(self.urdf_file, 'r') as f:
                urdf_content = f.read()

            # Replace package://my_package/ with the absolute path
            urdf_content = urdf_content.replace('package://' + pkg_name + '/', 'file://' + self.path_pkg + '/') 

            # Write to a temporary file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as temp_file:
                temp_file.write(urdf_content)
                self.temp_urdf_path = temp_file.name

            tree = ET.parse(self.temp_urdf_path)
            self.root_urdf = tree.getroot()

            urdf_link_names = self.urdf_import_func.get_urdf_link_names(self.root_urdf)
            self.ui.comboBox_import.addItems(urdf_link_names)

    def import_urdf(self):

        self.parse_urdf()
        self.set_gui()

    def set_gui(self):

        self.current_urdf_link = self.ui.comboBox_import.currentText()
        index = self.ui.comboBox_import.findText(self.current_urdf_link)

        if index == 0:
            bool_value = True
        else:
            bool_value = False
            
        self.ui.type_label_import.setVisible(not bool_value)
        self.ui.parent_label_import.setVisible(not bool_value)
        self.ui.mimic_label_import.setVisible(not bool_value)
        self.ui.joint_box_import.setEnabled(not bool_value)
        self.ui.joint_type_line_import.setVisible(not bool_value)
        self.ui.parent_line_import.setVisible(not bool_value)
        self.ui.checkBox_mimic_import.setVisible(not bool_value)
        
        self.ui.checkBox_base_import.setVisible(bool_value)
        if self.current_urdf_link:
            self.urdf_import_gui.read_urdf(self.current_urdf_link, self.root_urdf)
            self.urdf_import_gui.set_user_interface(self.urdf_file, self.path_pkg)

    def modify_robot_description(self):
        
        self.urdf_import_gui.read_urdf(self.current_urdf_link, self.root_urdf)

        replace_with = self.ui.comboBox_replace.currentText()
        index = self.ui.comboBox_import.findText(replace_with)
        self.robot_description_xacro.parent = self.ui.comboBox_newParent.currentText()

        if index == 0:
            macro = 'base'
        else:
            macro = 'part'
        
        if replace_with == 'new':
            self.robot_description.parse_robot_description_template()
            name_attr = 'new link'
            self.robot_description_xacro.name = self.current_urdf_link
            root = self.robot_description.root_template
            self.robot_description_xacro.set_xacro(root, name_attr, macro)
            self.robot_description.root.append(self.robot_description_xacro.element)
        else:
            name_attr = replace_with
            root = self.robot_description.root
            self.robot_description_xacro.set_xacro(root, name_attr, macro)
        
        self.robot_description.robot_description_func.write_robot_description(self.robot_description.tree)
        self.morpheus.xacro_to_urdf()
        self.morpheus.update_robot_description()
        index = self.ui.comboBox.count() - 1
        self.ui.comboBox.insertItem(index, self.ui.comboBox_import.currentText())
        self.ui.comboBox_replace.addItem(self.ui.comboBox_import.currentText())
        self.ui.comboBox_newParent.addItem(self.ui.comboBox_import.currentText())

    def show_urdf_import(self):

        cmd_robot_state_import = ['ros2', 'run', 'robot_state_publisher',
                           'robot_state_publisher', self.temp_urdf_path,
                           '--ros-args', '-r', '__node:=robot_state_publisher_import', 
                           '-r', 'robot_description:=robot_description_import',
                           '-r', 'joint_states:=joint_states_import']
        self.process_robot_state_import = subprocess.Popen(
            cmd_robot_state_import,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  
        )    
        cmd_joint_state_import = ['ros2', 'run', 'joint_state_publisher','joint_state_publisher',
                           '--ros-args', '-r', '__node:=joint_state_publisher_import', 
                           '-r', 'robot_description:=robot_description_import',
                           '-r', 'joint_states:=joint_states_import']
        self.process_joint_state_import = subprocess.Popen(
            cmd_joint_state_import,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        config_file = '/home/oscar/Dokumente/Masterarbeit/morpheus_ws/src/morpheus_description/user_interface/config/urdf_import.rviz' # TODO: generic
        cmd_rviz_import = ['ros2', 'run', 'rviz2', 'rviz2', '-d', config_file, '--ros-args', '-r', '__node:=rviz_import']
        self.process_rviz_import = subprocess.Popen(
            cmd_rviz_import,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )

        self.import_visualization = True

    def stop_visualizer(self):

        if self.import_visualization == True:

            os.killpg(self.process_robot_state_import.pid, signal.SIGINT)
            os.killpg(self.process_joint_state_import.pid, signal.SIGINT)
            os.killpg(self.process_rviz_import.pid, signal.SIGINT)
            # if self.process_joint_state_gui is not None:
            #     os.killpg(self.process_joint_state_gui.pid, signal.SIGINT)
            print('Processes ended...')

            self.import_visualization = False

    """
    --------------------------------------------------------------------------------------------------------
    -------------------------------------GUI related Functions----------------------------------------------
    --------------------------------------------------------------------------------------------------------
    """

    def event_initialization(self):

        # URDF Import Buttons
        self.ui.import_button.clicked.connect(self.import_urdf)
        self.ui.view_urdf_button.clicked.connect(self.show_urdf_import)
        self.ui.add_button_import.clicked.connect(self.modify_robot_description)
        self.ui.checkBox_base_import.setVisible(False)
        self.ui.checkBox_base_import.toggled.connect(self.base_properties)
        self.shape_selector()
        self.ui.radioButton_visual_2.toggled.connect(self.shape_selector)
        self.ui.radioButton_collision_2.toggled.connect(self.shape_selector)
        self.ui.radioButton_box_viz_2.toggled.connect(self.shape_selector)
        self.ui.radioButton_sphere_viz_2.toggled.connect(self.shape_selector)
        self.ui.radioButton_cylinder_viz_2.toggled.connect(self.shape_selector)
        self.ui.radioButton_mesh_viz_2.toggled.connect(self.shape_selector)
        self.ui.radioButton_box_col_2.toggled.connect(self.shape_selector)
        self.ui.radioButton_sphere_col_2.toggled.connect(self.shape_selector)
        self.ui.radioButton_cylinder_col_2.toggled.connect(self.shape_selector)
        self.ui.radioButton_mesh_col_2.toggled.connect(self.shape_selector)

        self.ui.comboBox_import.currentTextChanged.connect(self.set_gui)

    def comboBox_initialization(self):

        self.ui.comboBox_newParent.clear()
        self.ui.comboBox_newParent.addItems(self.robot_description.links_list)

        replace_list = ["new"]
        replace_list.extend(self.robot_description.links_list)
        self.ui.comboBox_replace.clear()
        self.ui.comboBox_replace.addItems(replace_list)

    def shape_selector(self):

        self.ui.groupBox_shape_viz_2.setVisible(self.ui.radioButton_visual_2.isChecked())
        self.ui.groupBox_shape_col_2.setVisible(self.ui.radioButton_collision_2.isChecked())

        if self.ui.radioButton_visual_2.isChecked():
            self.ui.groupBox_sphere_col_2.setVisible(False)
            self.ui.groupBox_mesh_col_2.setVisible(False)
            self.ui.groupBox_cylinder_col_2.setVisible(False)
            self.ui.groupBox_box_col_2.setVisible(False)
            self.ui.groupBox_sphere_viz_2.setVisible(self.ui.radioButton_sphere_viz_2.isChecked())
            self.ui.groupBox_mesh_viz_2.setVisible(self.ui.radioButton_mesh_viz_2.isChecked())
            self.ui.groupBox_cylinder_viz_2.setVisible(self.ui.radioButton_cylinder_viz_2.isChecked())
            self.ui.groupBox_box_viz_2.setVisible(self.ui.radioButton_box_viz_2.isChecked())
        else:
            self.ui.groupBox_sphere_col_2.setVisible(self.ui.radioButton_sphere_col_2.isChecked())
            self.ui.groupBox_mesh_col_2.setVisible(self.ui.radioButton_mesh_col_2.isChecked())
            self.ui.groupBox_cylinder_col_2.setVisible(self.ui.radioButton_cylinder_col_2.isChecked())
            self.ui.groupBox_box_col_2.setVisible(self.ui.radioButton_box_col_2.isChecked())
            self.ui.groupBox_sphere_viz_2.setVisible(False)
            self.ui.groupBox_mesh_viz_2.setVisible(False)
            self.ui.groupBox_cylinder_viz_2.setVisible(False)
            self.ui.groupBox_box_viz_2.setVisible(False)    

    def base_properties(self):
        if self.ui.checkBox_base_import.isChecked():
            self.ui.link_box_import.setEnabled(False)
            self.ui.mass_line_import.setEnabled(False)
        else:
            self.ui.link_box_import.setEnabled(True)
            self.ui.mass_line_import.setEnabled(True)


