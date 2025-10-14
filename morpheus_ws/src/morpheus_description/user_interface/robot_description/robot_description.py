import xml.etree.ElementTree as ET
import os

from robot_description.packages.functions import RobotDescriptionFunctions
from robot_description.packages.xacro import RobotDescriptionXacro
from robot_description.packages.user_interface import RobotDescriptionGUI

class RobotDescription:
    
    def __init__(self, ui, morpheus, path):

        self.ui = ui
        self.morpheus = morpheus
        self.package_path = path
        self.xacro_tag = '{http://www.ros.org/wiki/xacro}'
        
        self.parse_robot_description()
        self.parse_robot_description_template()

        self.robot_description_gui = RobotDescriptionGUI(self.ui, self.root, self.package_path)
        self.robot_description_xacro = RobotDescriptionXacro(self.ui, self.package_path)
        self.robot_description_func = RobotDescriptionFunctions(self.ui, self.root, self.robot_description_path,self.package_path)

        self.comboBox_initialization()
        self.event_initializiation()

        self.set_gui()

    """
    --------------------------------------------------------------------------------------------------------
    -------------------------------------Xacro related Functions--------------------------------------------
    --------------------------------------------------------------------------------------------------------
    """

    def parse_robot_description(self):
        self.robot_description_path = os.path.join(self.package_path, 'urdf/01_robot_description/robot_description.xacro') 
        self.tree = ET.parse(self.robot_description_path)
        self.root = self.tree.getroot()

    def parse_robot_description_template(self):
        self.robot_description_template_path = os.path.join(self.package_path, 'urdf/04_templates/robot_description.template.xacro')
        self.tree_template = ET.parse(self.robot_description_template_path)
        self.root_template = self.tree_template.getroot()

    def set_gui(self):

        self.current_link = self.ui.comboBox.currentText()
        index = self.ui.comboBox.findText(self.current_link)

        if index == 0:
            print('ji')
            bool_value = True
            self.base_properties()
            self.ui.remove_button.setEnabled(False)
        else:
            print('fff')
            bool_value = False
            self.ui.mass_line.setEnabled(True)
            self.ui.link_box.setEnabled(True)
            self.ui.remove_button.setEnabled(True)

        self.ui.type_label.setVisible(not bool_value)
        self.ui.parent_label.setVisible(not bool_value)
        self.ui.mimic_label.setVisible(not bool_value)
        self.ui.joint_box.setEnabled(not bool_value)
        self.ui.comboBox_joints.setVisible(not bool_value)
        self.ui.comboBox_parent.setVisible(not bool_value)
        self.ui.checkBox_mimic.setVisible(not bool_value)

        self.ui.checkBox_base.setVisible(bool_value)
        #self.ui.name_line.setReadOnly(bool_value)

        # setting up the combo boxes according to the current components of self.comboBox
        AllItems = [self.ui.comboBox.itemText(i) for i in range(self.ui.comboBox.count()-1)]
        self.ui.comboBox_parent.clear()
        self.ui.comboBox_parent.addItems(AllItems)

        moveable_links = self.robot_description_func.get_links_with_moveable_joints()
        self.ui.comboBox_mimic.clear()
        self.ui.comboBox_mimic.addItems(moveable_links)

        # remove selected item from parent and mimic combo Box since it is not possible
        # for a link to mimic or parent itself
        remove_link = self.current_link
        index_remove = self.ui.comboBox_parent.findText(remove_link)
        self.ui.comboBox_parent.removeItem(index_remove)
        index_remove = self.ui.comboBox_mimic.findText(remove_link)
        self.ui.comboBox_mimic.removeItem(index_remove)

        if self.current_link:
            if index == 0:
                self.macro = 'base'
            else:
                self.macro = 'part'
            self.robot_description_gui.read_xacro(self.current_link, self.macro)
            self.robot_description_gui.set_user_interface()

    def modify_robot_description(self):

        current_name = self.robot_description_gui.name
        new_name = self.ui.name_line.text()

        all_links = [self.ui.comboBox.itemText(i) for i in range(self.ui.comboBox.count())]
        if new_name in all_links and new_name != current_name:
            self.robot_description_func.dupes_warning()
        else:
            self.robot_description_xacro.read_user_interface()
            self.robot_description_xacro.set_xacro(self.root, self.current_link, self.macro)
            self.robot_description_func.write_robot_description(self.tree)
            self.morpheus.xacro_to_urdf()
            self.morpheus.update_robot_description()

            self.robot_description_func.get_missing_interfaces()

    def remove_link(self):

        new_parent = self.ui.comboBox.itemText(0)
        
        for self.element in self.root.iter(self.xacro_tag + 'part'):
            if self.element.attrib.get('parent') == self.current_link:
                self.element.set('parent', new_parent)

        for self.element in self.root.iter(self.xacro_tag + 'part'):
            if self.element.attrib.get('name') == self.current_link:
                self.root.remove(self.element)

        self.robot_description_func.write_robot_description(self.tree)
        self.morpheus.xacro_to_urdf()
        self.morpheus.update_robot_description()

        remove_link = self.current_link
        index = self.ui.comboBox.findText(remove_link)
        self.ui.comboBox.removeItem(index)
        index = self.ui.camera_reference.findText(remove_link)
        self.ui.camera_reference.removeItem(index)
        self.ui.lidar_reference.removeItem(index)
        self.ui.imu_reference.removeItem(index)
        self.ui.ft_reference.removeItem(index)

        index = self.ui.comboBox_ros2control_missing.findText(remove_link)
        self.ui.comboBox_ros2control_missing.removeItem(index)
        index = self.ui.comboBox_ros2control_existing.findText(remove_link)

        self.robot_description_func.remove_sensor(remove_link)

        if index != -1:
            self.ui.comboBox_ros2control_existing.setCurrentText(remove_link)
            self.robot_description_func.remove_joint()

        index = self.ui.comboBox_newParent.findText(remove_link)
        self.ui.comboBox_newParent.removeItem(index)
        index = self.ui.comboBox_replace.findText(remove_link)
        self.ui.comboBox_replace.removeItem(index)

    def add_link(self):
        all_links = [self.ui.comboBox.itemText(i) for i in range(self.ui.comboBox.count())]
        if self.ui.name_line.text() in all_links:
            self.robot_description_func.dupes_warning()
        else:
            index = self.ui.comboBox.count() - 1
            new_link_name = self.ui.name_line.text()
            self.parse_robot_description_template()
            root = self.root_template

            self.robot_description_xacro.read_user_interface()
            self.robot_description_xacro.set_xacro(root, 'new link', self.macro)
            self.root.append(self.robot_description_xacro.element)
            self.robot_description_func.write_robot_description(self.tree)
            self.morpheus.xacro_to_urdf()
            self.morpheus.update_robot_description()

            self.ui.comboBox.insertItem(index, new_link_name)
            self.ui.comboBox.setCurrentText(new_link_name)
            self.ui.comboBox_parent.addItem(new_link_name)
            self.ui.camera_reference.addItem(new_link_name)
            self.ui.lidar_reference.addItem(new_link_name)
            self.ui.imu_reference.addItem(new_link_name)
            self.ui.ft_reference.addItem(new_link_name)
            self.ui.comboBox_newParent.addItem(new_link_name)
            self.ui.comboBox_replace.addItem(new_link_name)

            self.robot_description_func.get_missing_interfaces()

    """
    --------------------------------------------------------------------------------------------------------
    -------------------------------------GUI related Functions----------------------------------------------
    --------------------------------------------------------------------------------------------------------
    """

    def event_initializiation(self):

        # Xacro Manipulation
        self.ui.preview_button.clicked.connect(self.modify_robot_description)
        self.ui.remove_button.clicked.connect(self.remove_link)
        self.ui.add_button.clicked.connect(self.add_link)

        # Joint State Visualizer
        self.ui.joint_gui_button.clicked.connect(self.morpheus.start_joint_GUI)

        # Link Group
        self.ui.select_mesh_viz.clicked.connect(self.robot_description_func.get_mesh_file_viz) # Mesh File Selection
        self.ui.select_mesh_col.clicked.connect(self.robot_description_func.get_mesh_file_col) # Mesh File Selection
        self.ui.calculateInertiaButton.clicked.connect(self.robot_description_func.calculate_inertial_tag) # Calculation Moment of Inertia
        self.ui.toolButton_color.clicked.connect(self.robot_description_func.color_picker) # Color Picker
        
        # Shape Selector for either Visual or Collision
        self.shape_selection()
        self.ui.radioButton_visual.toggled.connect(self.shape_selection) 
        self.ui.radioButton_collision.toggled.connect(self.shape_selection)
        self.ui.radioButton_box_viz.toggled.connect(self.shape_selection)
        self.ui.radioButton_sphere_viz.toggled.connect(self.shape_selection)
        self.ui.radioButton_cylinder_viz.toggled.connect(self.shape_selection)
        self.ui.radioButton_mesh_viz.toggled.connect(self.shape_selection)
        self.ui.radioButton_box_col.toggled.connect(self.shape_selection)
        self.ui.radioButton_sphere_col.toggled.connect(self.shape_selection)
        self.ui.radioButton_cylinder_col.toggled.connect(self.shape_selection)
        self.ui.radioButton_mesh_col.toggled.connect(self.shape_selection)

        # Link Attributes
        self.ui.checkBox_base.toggled.connect(self.base_properties) # Disable Link Properties if checked
        self.ui.checkBox_mimic.toggled.connect(self.enable_mimic_group) # Enable Mimic Group if checked
        self.enable_mimic_group()

        # Copy and Paste 
        self.ui.toolButton_copy.clicked.connect(self.robot_description_func.copy_func)
        self.ui.toolButton_paste.clicked.connect(self.robot_description_func.paste_func)

        # Reset the GUI Value Fields if the selected Link gets changed
        self.ui.comboBox.currentTextChanged.connect(self.set_gui)

    def comboBox_initialization(self):

        self.links_list = self.robot_description_func.get_link_names(self.root)
        links_list = self.links_list 

        self.ui.comboBox.clear()
        self.ui.comboBox.addItems(links_list)
        self.ui.comboBox.addItem('new link')
        self.current_link = self.ui.comboBox.currentText()

        self.ui.comboBox_parent.clear()
        self.ui.comboBox_parent.addItems(links_list)

        joint_types = ["fixed", "revolute", "prismatic", "continuous", "floating", "planar"]
        self.ui.comboBox_joints.addItems(joint_types)

    def shape_selection(self):

        self.ui.groupBox_shape_viz.setVisible(self.ui.radioButton_visual.isChecked())
        self.ui.groupBox_shape_col.setVisible(self.ui.radioButton_collision.isChecked())
        self.ui.groupBox_origin_viz.setVisible(self.ui.radioButton_visual.isChecked())
        self.ui.groupBox_origin_col.setVisible(self.ui.radioButton_collision.isChecked())
        
        if self.ui.radioButton_visual.isChecked():

            #self.calculateInertiaButton.setEnabled(False) # anpassen sodass alle gehen
            self.ui.groupBox_sphere_viz.setVisible(self.ui.radioButton_sphere_viz.isChecked())
            self.ui.groupBox_mesh_viz.setVisible(self.ui.radioButton_mesh_viz.isChecked())
            self.ui.groupBox_cylinder_viz.setVisible(self.ui.radioButton_cylinder_viz.isChecked())
            self.ui.groupBox_box_viz.setVisible(self.ui.radioButton_box_viz.isChecked())

            self.ui.groupBox_sphere_col.setVisible(False)
            self.ui.groupBox_mesh_col.setVisible(False)
            self.ui.groupBox_cylinder_col.setVisible(False)
            self.ui.groupBox_box_col.setVisible(False)
        else:
            self.ui.groupBox_sphere_col.setVisible(self.ui.radioButton_sphere_col.isChecked())
            self.ui.groupBox_mesh_col.setVisible(self.ui.radioButton_mesh_col.isChecked())
            self.ui.groupBox_cylinder_col.setVisible(self.ui.radioButton_cylinder_col.isChecked())
            self.ui.groupBox_box_col.setVisible(self.ui.radioButton_box_col.isChecked())
            self.ui.groupBox_sphere_viz.setVisible(False)
            self.ui.groupBox_mesh_viz.setVisible(False)
            self.ui.groupBox_cylinder_viz.setVisible(False)
            self.ui.groupBox_box_viz.setVisible(False)

    def base_properties(self):
        print(self.ui.checkBox_base.isChecked())
        if self.ui.checkBox_base.isChecked():
            self.ui.link_box.setEnabled(False)
            self.ui.mass_line.setEnabled(False)
        else:
            self.ui.link_box.setEnabled(True)
            self.ui.mass_line.setEnabled(True)

    def enable_mimic_group(self):
        if self.ui.checkBox_mimic.isChecked():
            self.ui.groupBox_mimic.setEnabled(True)
        else:
            self.ui.groupBox_mimic.setEnabled(False)