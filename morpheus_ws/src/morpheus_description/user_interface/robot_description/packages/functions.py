import xml.etree.ElementTree as ET
import pymeshlab
import numpy as np
import os
from pyquaternion import Quaternion

from PyQt6.QtWidgets import QFileDialog, QColorDialog, QMessageBox

class RobotDescriptionFunctions:

    def __init__(self, ui, root, robot_description_path, path):
    
        self.ui = ui
        self.root = root
        self.robot_description_path = robot_description_path
        self.package_path = path
        self.xacro_tag = '{http://www.ros.org/wiki/xacro}'

    def copy_func(self):

        # Attributes
        self.name_copy = self.ui.name_line.text()
        self.mass_copy = self.ui.mass_line.value()
        self.joint_type_copy = self.ui.comboBox_joints.currentText()
        self.parent_copy = self.ui.comboBox_parent.currentText()
        self.mimic_copy = self.ui.checkBox_mimic.isChecked()
        if self.mimic_copy == True:
            self.mimic_copy = "1"
        else:
            self.mimic_copy = "0"
            
        self.empty_copy = self.ui.checkBox_base.isChecked()
        if self.empty_copy == True:
            self.empty_copy = "1"
        else:
            self.empty_copy = "0"
            
        # Visual Tag
        #self.shape_type_viz_copy = self.shape_type_viz
        self.link_viz_x_copy = self.ui.link_origin_x_viz.value()
        self.link_viz_y_copy = self.ui.link_origin_y_viz.value()
        self.link_viz_z_copy = self.ui.link_origin_z_viz.value()
        #self.link_viz_xyz_copy = self.link_viz_x_copy + " " + self.link_viz_y_copy + " " + self.link_viz_z_copy

        self.link_viz_roll_copy  = np.deg2rad(self.ui.link_origin_r_viz.value())
        self.link_viz_pitch_copy = np.deg2rad(self.ui.link_origin_p_viz.value())
        self.link_viz_yaw_copy   = np.deg2rad(self.ui.link_origin_yaw_viz.value())
        #self.link_viz_rpy_copy = self.link_viz_roll_copy + " " + self.link_viz_pitch_copy + " " + self.link_viz_yaw_copy

        if self.ui.radioButton_box_viz.isChecked():
            self.new_tag_copy = 'box'
            self.new_attr_name_copy = 'size'
            self.link_viz_box_x_copy = self.ui.box_x_viz.value()
            self.link_viz_box_y_copy = self.ui.box_y_viz.value()
            self.link_viz_box_z_copy = self.ui.box_z_viz.value()
            #self.link_viz_box_copy = self.link_viz_box_x_copy + " " + self.link_viz_box_y_copy + " " + self.link_viz_box_z_copy
        elif self.ui.radioButton_sphere_viz.isChecked():
            self.new_tag_copy = 'sphere'
            self.new_attr_name_copy = 'radius'
            self.link_viz_sph_r_copy = self.ui.radius_sph_viz.value()
        elif self.ui.radioButton_cylinder_viz.isChecked():
            self.new_tag_copy = 'cylinder'
            self.new_attr_name_r_copy = 'radius'
            self.new_attr_name_l_copy = 'length'
            self.link_viz_cyl_r_copy = self.ui.radius_cyl_viz.value()
            self.link_viz_cyl_l_copy = self.ui.length_cyl_viz.value()
        elif self.ui.radioButton_mesh_viz.isChecked():
            self.new_tag_copy = 'mesh'
            self.new_attr_name_copy = 'filename'
            if "package://" in self.ui.viz_mesh_line.text():
                self.link_viz_mesh_copy = self.ui.viz_mesh_line.text()
            elif "file://" in self.ui.viz_mesh_line.text():
                self.link_viz_mesh_copy = self.ui.viz_mesh_line.text()
            else:
                self.link_viz_mesh_copy = "file://" + self.ui.viz_mesh_line.text()

        red = self.ui.toolButton_color.palette().base().color().getRgb()[0]/255
        green = self.ui.toolButton_color.palette().base().color().getRgb()[1]/255
        blue = self.ui.toolButton_color.palette().base().color().getRgb()[2]/255
        self.rgb_copy = str(red) + " " + str(green) + " " + str(blue) + " 1.0"

        # Collision Tag
        #self.shape_type_col_copy = self.shape_type_col
        self.link_col_x_copy = self.ui.link_origin_x_col.value()
        self.link_col_y_copy = self.ui.link_origin_y_col.value()
        self.link_col_z_copy = self.ui.link_origin_z_col.value()
        #self.link_col_xyz_copy = self.link_col_x_copy + " " + self.link_col_y_copy + " " + self.link_col_z_copy

        self.link_col_roll_copy  = np.deg2rad(self.ui.link_origin_r_col.value())
        self.link_col_pitch_copy = np.deg2rad(self.ui.link_origin_p_col.value())
        self.link_col_yaw_copy   = np.deg2rad(self.ui.link_origin_yaw_col.value())
        #self.link_col_rpy_copy = self.link_col_roll_copy + " " + self.link_col_pitch_copy + " " + self.link_col_yaw_copy

        if self.ui.radioButton_box_col.isChecked():
            self.new_tag_col_copy = 'box'
            self.new_attr_name_col_copy = 'size'
            self.link_col_box_x_copy = self.ui.box_x_col.value()
            self.link_col_box_y_copy = self.ui.box_y_col.value()
            self.link_col_box_z_copy = self.ui.box_z_col.value()
            #self.link_col_box_copy = self.link_col_box_x_copy + " " + self.link_col_box_y_copy + " " + self.link_col_box_z_copy
        elif self.ui.radioButton_sphere_col.isChecked():
            self.new_tag_col_copy = 'sphere'
            self.new_attr_name_col_copy = 'radius'
            self.link_col_sph_r_copy = self.ui.radius_sph_col.value()
        elif self.ui.radioButton_cylinder_col.isChecked():
            self.new_tag_col_copy = 'cylinder'
            self.new_attr_name_r_col_copy = 'radius'
            self.new_attr_name_l_col_copy = 'length'
            self.link_col_cyl_r_copy = self.ui.radius_cyl_col.value()
            self.link_col_cyl_l_copy = self.ui.length_cyl_col.value()
        elif self.ui.radioButton_mesh_col.isChecked():
            self.new_tag_col_copy = 'mesh'
            self.new_attr_name_col_copy = 'filename'
            if "package://" in self.ui.col_mesh_line.text():
                self.link_col_mesh_copy = self.ui.col_mesh_line.text()
            elif "file://" in self.ui.col_mesh_line.text():
                self.link_col_mesh_copy = self.ui.col_mesh_line.text()
            else:
                self.link_col_mesh_copy = "file://" + self.ui.col_mesh_line.text()
        
        # Inertial Tag
        self.link_com_x_copy = self.ui.link_cof_x.value()
        self.link_com_y_copy = self.ui.link_cof_y.value()
        self.link_com_z_copy = self.ui.link_cof_z.value()
        #self.link_com_copy = self.link_com_x_copy + " " + self.link_com_y_copy + " " + self.link_com_z_copy
        
        self.link_xx_copy = self.ui.link_moi_xx.value()
        self.link_yy_copy = self.ui.link_moi_yy.value()
        self.link_zz_copy = self.ui.link_moi_zz.value()
        self.link_xy_copy = self.ui.link_moi_xy.value()
        self.link_xz_copy = self.ui.link_moi_xz.value()
        self.link_yz_copy = self.ui.link_moi_yz.value()

        self.joint_x_copy = self.ui.joint_origin_x.value()
        self.joint_y_copy = self.ui.joint_origin_y.value()
        self.joint_z_copy = self.ui.joint_origin_z.value()
        #self.joint_xyz_copy = self.joint_x_copy + " " + self.joint_y_copy + " " + self.joint_z_copy           
        
        self.joint_roll_copy = np.deg2rad(self.ui.joint_origin_r.value())
        self.joint_pitch_copy = np.deg2rad(self.ui.joint_origin_p.value())
        self.joint_yaw_copy = np.deg2rad(self.ui.joint_origin_yaw.value())
        #self.joint_rpy = self.joint_roll_copy + " " + self.joint_pitch_copy + " " + self.joint_yaw_copy     
        
        self.joint_aor_x_copy = self.ui.joint_aor_x.value()
        self.joint_aor_y_copy = self.ui.joint_aor_y.value()
        self.joint_aor_z_copy = self.ui.joint_aor_z.value()
        #self.joint_aor =  self.joint_aor_x_copy + " " + self.joint_aor_y_copy + " " + self.joint_aor_z_copy  
        
        self.joint_effort_copy = self.ui.joint_limit_effort.value()
        self.joint_velocity_copy = self.ui.joint_limit_vel.value()
        self.joint_upper_copy = self.ui.joint_limit_lower.value()
        self.joint_lower_copy = self.ui.joint_limit_upper.value()

        self.joint_damping_copy = self.ui.joint_damping.value()
        self.joint_friction_copy = self.ui.joint_friction.value()

        self.joint_multiplier_copy = self.ui.joint_mimic_mult.value()
        self.joint_offset_copy = self.ui.joint_mimic_offset.value()
        self.joint_mimic_joint_copy = self.ui.comboBox_mimic.currentText()

    def paste_func(self):
        
        self.ui.name_line.setText(self.name_copy)
        self.ui.mass_line.setValue(self.mass_copy)
        if self.ui.comboBox.currentIndex() != 0:
            self.ui.comboBox_parent.setCurrentText(self.parent_copy)
            self.ui.comboBox_joints.setCurrentText(self.joint_type_copy)
            self.ui.checkBox_mimic.setChecked(int(self.mimic_copy))
        else:
            self.ui.checkBox_base.setChecked(int(self.empty_copy))

        # Parsing the values from the xacro file to the UI
        self.ui.link_origin_x_viz.setValue(self.link_viz_x_copy)
        self.ui.link_origin_y_viz.setValue(self.link_viz_y_copy)
        self.ui.link_origin_z_viz.setValue(self.link_viz_z_copy)

        self.ui.link_origin_r_viz.setValue(np.rad2deg(self.link_viz_roll_copy))
        self.ui.link_origin_p_viz.setValue(np.rad2deg(self.link_viz_pitch_copy))
        self.ui.link_origin_yaw_viz.setValue(np.rad2deg(self.link_viz_yaw_copy))
        ###
        if self.new_tag_copy == 'mesh':
            self.ui.radioButton_mesh_viz.setChecked(True)
            self.ui.viz_mesh_line.setText(self.link_viz_mesh_copy)
        elif self.new_tag_copy == 'box':
            self.ui.radioButton_box_viz.setChecked(True)
            self.ui.box_x_viz.setValue(self.link_viz_box_x_copy)
            self.ui.box_y_viz.setValue(self.link_viz_box_y_copy)
            self.ui.box_z_viz.setValue(self.link_viz_box_z_copy)
        elif self.new_tag_copy == 'cylinder':
            self.ui.radioButton_cylinder_viz.setChecked(True)
            self.ui.length_cyl_viz.setValue(self.link_viz_cyl_l_copy)            
            self.ui.radius_cyl_viz.setValue(self.link_viz_cyl_r_copy)
        elif self.new_tag_copy == 'sphere':
            self.ui.radioButton_sphere_viz.setChecked(True)
            self.ui.radius_sph_viz.setValue(self.link_viz_sph_r_copy)

        self.ui.toolButton_color.setStyleSheet("background-color : " + self.rgb_copy)

        self.ui.link_origin_x_col.setValue(self.link_col_x_copy)
        self.ui.link_origin_y_col.setValue(self.link_col_y_copy)
        self.ui.link_origin_z_col.setValue(self.link_col_z_copy)

        self.ui.link_origin_r_col.setValue(np.rad2deg(self.link_col_roll_copy))
        self.ui.link_origin_p_col.setValue(np.rad2deg(self.link_col_pitch_copy))
        self.ui.link_origin_yaw_col.setValue(np.rad2deg(self.link_col_yaw_copy))
        ###
        if self.new_tag_col_copy == 'mesh':
            self.ui.radioButton_mesh_col.setChecked(True)
            self.ui.col_mesh_line.setText(self.link_col_mesh_copy)
        elif self.new_tag_col_copy == 'box':
            self.ui.radioButton_box_col.setChecked(True)
            self.ui.box_x_col.setValue(self.link_col_box_x_copy)
            self.ui.box_y_col.setValue(self.link_col_box_y_copy)
            self.ui.box_z_col.setValue(self.link_col_box_z_copy)
        elif self.new_tag_col_copy == 'cylinder':
            self.ui.radioButton_cylinder_col.setChecked(True)
            self.ui.length_cyl_col.setValue(self.link_col_cyl_l_copy)            
            self.ui.radius_cyl_col.setValue(self.link_col_cyl_r_copy)
        elif self.new_tag_col_copy == 'sphere':
            self.ui.radioButton_sphere_col.setChecked(True)
            self.ui.radius_sph_col.setValue(self.link_col_sph_r_copy)

        self.ui.link_cof_x.setValue(self.link_com_x_copy)
        self.ui.link_cof_y.setValue(self.link_com_y_copy)
        self.ui.link_cof_z.setValue(self.link_com_z_copy)

        self.ui.link_moi_xx.setValue(self.link_xx_copy)
        self.ui.link_moi_yy.setValue(self.link_yy_copy)
        self.ui.link_moi_zz.setValue(self.link_zz_copy)

        self.ui.link_moi_xy.setValue(self.link_xy_copy)
        self.ui.link_moi_xz.setValue(self.link_xz_copy)
        self.ui.link_moi_yz.setValue(self.link_yz_copy)

        #---------------------------------------------------------------------

        if self.ui.comboBox.currentIndex() != 0:
            self.ui.joint_origin_x.setValue(self.joint_x_copy)
            self.ui.joint_origin_y.setValue(self.joint_y_copy)
            self.ui.joint_origin_z.setValue(self.joint_z_copy)

            self.ui.joint_origin_r.setValue(np.rad2deg(self.joint_roll_copy))
            self.ui.joint_origin_p.setValue(np.rad2deg(self.joint_pitch_copy))
            self.ui.joint_origin_yaw.setValue(np.rad2deg(self.joint_yaw_copy))
            
            self.ui.joint_aor_x.setValue(self.joint_aor_x_copy)
            self.ui.joint_aor_y.setValue(self.joint_aor_y_copy)
            self.ui.joint_aor_z.setValue(self.joint_aor_z_copy)

            self.ui.joint_limit_effort.setValue(self.joint_effort_copy)
            self.ui.joint_limit_vel.setValue(self.joint_velocity_copy)
            self.ui.joint_limit_lower.setValue(self.joint_lower_copy)
            self.ui.joint_limit_upper.setValue(self.joint_upper_copy)

            self.ui.joint_damping.setValue(self.joint_damping_copy)
            self.ui.joint_friction.setValue(self.joint_friction_copy)

            self.ui.joint_mimic_mult.setValue(self.joint_multiplier_copy)
            self.ui.joint_mimic_offset.setValue(self.joint_offset_copy)
            self.ui.comboBox_mimic.setCurrentText(self.joint_mimic_joint_copy)

    def write_robot_description(self, tree):

        ET.register_namespace("xacro", 'http://www.ros.org/wiki/xacro')
        ET.indent(tree,'  ', level=0)
        tree.write(self.robot_description_path)
        
    def get_link_names(self, root):

        self.parts_list = []
        
        for self.element in root.iter(self.xacro_tag + 'base'):    
            self.parts_list.append(self.element.attrib.get('name'))

        for self.element in root.iter(self.xacro_tag + 'part'):    
            self.parts_list.append(self.element.attrib.get('name'))
 
        return self.parts_list
    
    def get_links_with_moveable_joints(self):

        self.moveable_links_list = []
        for self.element in self.root.iter(self.xacro_tag + 'part'):
            if self.element.attrib.get('type') != 'fixed':      
                self.moveable_links_list.append(self.element.attrib.get('name'))
        
        return self.moveable_links_list

    def get_mesh_file_viz(self):
        file_filer = 'STL File (*.stl *.STL);; DAE File (*.dae *.DAE);; All Mesh Files (*.stl *.STL *.dae *.DAE)'
        response = QFileDialog.getOpenFileName(
            parent=self.ui,
            caption='Select a mesh file',
            directory='/home/oscar/Dokumente/Masterarbeit/morpheus_ws/src/morpheus_description/urdf/01_robot_description/meshes', # TODO: Generic
            filter=file_filer,
            initialFilter='DAE File (*.dae *.DAE)'
        )

        if response[0]:
            self.ui.viz_mesh_line.setText(response[0])

    def get_mesh_file_col(self):
        file_filer = 'STL File (*.stl *.STL);; DAE File (*.dae *.DAE);; All Mesh Files (*.stl *.STL *.dae *.DAE)'
        response = QFileDialog.getOpenFileName(
            parent=self.ui,
            caption='Select a mesh file',
            directory='/home/oscar/Dokumente/Masterarbeit/morpheus_ws/src/morpheus_description/urdf/01_robot_description/meshes', # TODO: Generic
            filter=file_filer,
            initialFilter='STL File (*.stl *.STL)'
        )
        if response[0]:
            self.ui.col_mesh_line.setText(response[0])

    def calculate_inertial_tag(self):

        mass = self.ui.mass_line.value()

        scale_factor=100
        ms = pymeshlab.MeshSet()
        
        if self.ui.radioButton_box_viz.isChecked():

            Ixx = 1/12 * mass * (np.power(self.ui.box_y_viz.value(),2) + np.power(self.ui.box_z_viz.value(),2))
            Iyy = 1/12 * mass * (np.power(self.ui.box_x_viz.value(),2) + np.power(self.ui.box_z_viz.value(),2))
            Izz = 1/12 * mass * (np.power(self.ui.box_x_viz.value(),2) + np.power(self.ui.box_y_viz.value(),2))

            self.com_x = 0 + self.ui.link_origin_x_col.value() # Verschiebung des Links in x-Richtung
            self.com_y = 0 + self.ui.link_origin_y_col.value() # Verschiebung des Links in y-Richtung
            self.com_z = 0 + self.ui.link_origin_z_col.value() # Verschiebung des Links in z-Richtung

            Ixy = Ixz = Iyz = 0
            self.tensor = np.array([[Ixx, Ixy, Ixz],
                                    [Ixy, Iyy, Iyz],
                                    [Ixz, Iyz, Izz]])
        
        elif self.ui.radioButton_sphere_viz.isChecked():

            Ixx = 2/3 * mass * self.ui.radius_sph_viz.value()
            Iyy = 2/3 * mass * self.ui.radius_sph_viz.value()
            Izz = 2/3 * mass * self.ui.radius_sph_viz.value()

            self.com_x = 0 + self.ui.link_origin_x_col.value() # Verschiebung des Links in x-Richtung
            self.com_y = 0 + self.ui.link_origin_y_col.value() # Verschiebung des Links in y-Richtung
            self.com_z = 0 + self.ui.link_origin_z_col.value() # Verschiebung des Links in z-Richtung

            Ixy = Ixz = Iyz = 0
            self.tensor = np.array([[Ixx, Ixy, Ixz],
                                    [Ixy, Iyy, Iyz],
                                    [Ixz, Iyz, Izz]])
        
        elif self.ui.radioButton_cylinder_viz.isChecked():

            Ixx = 1/12 * mass * (3*np.power(self.ui.radius_cyl_viz.value(),2) + np.power(self.ui.length_cyl_viz.value(),2))
            Iyy = 1/12 * mass * (3*np.power(self.ui.radius_cyl_viz.value(),2) + np.power(self.ui.length_cyl_viz.value(),2))
            Izz = 1/2 * mass * self.ui.radius_cyl_viz.value()

            Ixy = Ixz = Iyz = 0

            self.tensor = np.array([[Ixx, Ixy, Ixz],
                                    [Ixy, Iyy, Iyz],
                                    [Ixz, Iyz, Izz]])

            self.com_x = 0 + self.ui.link_origin_x_col.value() # Verschiebung des Links in x-Richtung
            self.com_y = 0 + self.ui.link_origin_y_col.value() # Verschiebung des Links in y-Richtung
            self.com_z = 0 + self.ui.link_origin_z_col.value() # Verschiebung des Links in z-Richtung

        elif self.ui.radioButton_mesh_viz.isChecked(): #and '.dae' in str(self.ui.viz_mesh_line.text()).lower():

            file_name = str(self.ui.viz_mesh_line.text())

            if "package://" in file_name:
                file_name = file_name.replace("package://morpheus_description", self.package_path) # TODO make morpheus_description substr generic
            elif "file://" in file_name:
                file_name = file_name.replace("file://","")

            #self.viz_mesh_line.text()
            ms.load_new_mesh(file_name)

            # Fehler abfangen wenn file_name und mass == NaN sind

            # Calculating the Center of Mass
            geom = ms.get_geometric_measures()
            com = geom['barycenter']

            # Scaling
            ms.compute_matrix_from_scaling_or_normalization(axisx=scale_factor, axisy=scale_factor, axisz=scale_factor)

            # Generating the convex hull of the mesh
            ms.generate_convex_hull()  # TODO only if object is not watertight

            # Calculating intertia self.tensor
            geom = ms.get_geometric_measures()
            volume = geom['mesh_volume']
            self.tensor = geom['inertia_tensor'] / pow(scale_factor, 2) * mass / volume

            self.com_x = com[0] 
            self.com_y = com[1]
            self.com_z = com[2] 

        if self.ui.radioButton_mesh_viz.isChecked() and not self.ui.viz_mesh_line.text(): #or '.stl' in str(self.ui.viz_mesh_line.text()).lower():
            pass
        else:
            transformed_vector, transformed_inertia = self.transform_vector()

            self.ui.link_cof_x.setValue(transformed_vector[0])
            self.ui.link_cof_y.setValue(transformed_vector[1])
            self.ui.link_cof_z.setValue(transformed_vector[2])
        
            self.ui.link_moi_xx.setValue(transformed_inertia[0][0])
            self.ui.link_moi_yy.setValue(transformed_inertia[1][1])
            self.ui.link_moi_zz.setValue(transformed_inertia[2][2])

            self.ui.link_moi_xz.setValue(transformed_inertia[2][0])
            self.ui.link_moi_xy.setValue(transformed_inertia[1][0])
            self.ui.link_moi_yz.setValue(transformed_inertia[1][2])

    def transform_vector(self):
        
        # Define Rotation Angles
        x_rotation_angle = np.deg2rad(self.ui.link_origin_r_viz.value())
        y_rotation_angle = np.deg2rad(self.ui.link_origin_p_viz.value())
        z_rotation_angle = np.deg2rad(self.ui.link_origin_yaw_viz.value())
        
        # Create quaternions for rotations around x, y, z axes
        qx = Quaternion(axis=[1, 0, 0], angle=x_rotation_angle)  # X-axis rotation
        qy = Quaternion(axis=[0, 1, 0], angle=y_rotation_angle)  # Y-axis rotation
        qz = Quaternion(axis=[0, 0, 1], angle=z_rotation_angle)  # Z-axis rotation

        # Combine rotations (order: z * y * x)
        q_combined = qz * (qy * qx)

        R_q = q_combined.normalised.rotation_matrix

        # Initial Vector: COM   
        vector_mass = np.array([self.com_x, self.com_y, self.com_z])

        # Convert COM Vector to a pure Quaternion 
        vec_quat = np.array([0, *vector_mass])

        # COM rotation: q * v * q^-1
        q_conj = q_combined.conjugate
        rotation = (q_combined * vec_quat) * q_conj

        # Inertia self.tensor Rotation
        transformend_inertia_tensor = np.matmul(np.matmul(R_q, self.tensor),np.transpose(R_q))

        # Apply translation
        translation = np.array([self.ui.link_origin_x_viz.value(), 
                                self.ui.link_origin_y_viz.value(), 
                                self.ui.link_origin_z_viz.value()])
        transformed_vector = rotation.vector + translation

        return transformed_vector, transformend_inertia_tensor


    def color_picker(self):
        color = QColorDialog.getColor()
        if color.isValid():
            self.ui.toolButton_color.setStyleSheet("background-color : " + color.name())

    def dupes_warning(self):

        msg = QMessageBox()
        msg.setIcon(msg.Icon.Warning)
        msg.setStyleSheet("QLabel{min-height:100 px} QLabel{subcontrol-position: top center}")
        msg.setText("There is a Duplicate! Please choose a valid name for the link first!        ")
        msg.setWindowTitle("Warning MessageBox")
        msg.setStandardButtons(msg.StandardButton.Ok)
        msg.exec()

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
        self.ros2_control_path = os.path.join(self.package_path, 'urdf/03_ros2_control/ros2_control.xacro')
        self.tree_ros2control = ET.parse(self.ros2_control_path)
        self.root_ros2control = self.tree_ros2control.getroot()
        joint_list = []
        for self.element in self.root_ros2control.iter('joint'):
            joint_name = self.element.attrib.get('name')
            joint_name = joint_name.replace('_joint','')
            joint_list.append(joint_name)
        return joint_list
    
    def get_links_with_moveable_joints(self):

        self.moveable_links_list = []
        for self.element in self.root.iter(self.xacro_tag + 'part'):
            if self.element.attrib.get('type') != 'fixed':      
                self.moveable_links_list.append(self.element.attrib.get('name'))
        
        return self.moveable_links_list
    
    def remove_joint(self):
        self.ros2_control_path = os.path.join(self.package_path, 'urdf/03_ros2_control/ros2_control.xacro')
        self.tree_ros2control = ET.parse(self.ros2_control_path)
        self.root_ros2control = self.tree_ros2control.getroot()
        current_joint = self.ui.comboBox_ros2control_existing.currentText()
        for self.element in self.root_ros2control.iter('joint'):
            if self.element.attrib.get('name') == str(current_joint + '_joint'):
                self.root_ros2control[0].remove(self.element)

                ET.register_namespace("xacro", 'http://www.ros.org/wiki/xacro')
                ET.indent(self.tree_ros2control,'  ', level=0)
                self.tree_ros2control.write(self.ros2_control_path)

                index = self.ui.comboBox_ros2control_existing.findText(current_joint)
                self.ui.comboBox_ros2control_existing.removeItem(index)
                index = self.ui.comboBox.findText(current_joint)
                if index !=-1:
                    self.ui.comboBox_ros2control_missing.addItem(current_joint)

    def remove_sensor(self, current_link):
        self.gazebo_sensors_path = os.path.join(self.package_path, 'urdf/02_gazebo/gazebo.xacro')
        self.tree_gazebo = ET.parse(self.gazebo_sensors_path)
        self.root_gazebo = self.tree_gazebo.getroot()
        
        all_cameras = [self.ui.camera_select.itemText(i) for i in range(self.ui.camera_select.count()-1)]
        all_lidars = [self.ui.lidar_select.itemText(i) for i in range(self.ui.lidar_select.count()-1)]
        all_imus = [self.ui.imu_select.itemText(i) for i in range(self.ui.imu_select.count()-1)]
        all_fts = [self.ui.ft_select.itemText(i) for i in range(self.ui.ft_select.count()-1)]

        all_sensors = all_cameras + all_lidars + all_imus + all_fts

        for sensor in all_sensors:
            sensor_select = sensor.split()
            sensor_name = sensor_select[0]
            sensor_reference = sensor_select[2]
            print(sensor)
            print(sensor_name)
            print(sensor_reference)
            if current_link == sensor_reference.replace('_link','') or current_link == sensor_reference.replace('_joint',''):
                self.element = self.root_gazebo.find(".//sensor[@name='" + sensor_name + "']/..[@reference='" + sensor_reference + "']")
                self.root_gazebo.remove(self.element)
                ET.indent(self.tree_gazebo,'  ', level=0)
                ET.register_namespace("xacro", 'http://www.ros.org/wiki/xacro')
                self.tree_gazebo .write(self.gazebo_sensors_path, 'UTF-8')

                index = self.ui.camera_select.findText(sensor)
                self.ui.camera_select.removeItem(index)
                index = self.ui.lidar_select.findText(sensor)
                self.ui.lidar_select.removeItem(index)
                index = self.ui.imu_select.findText(sensor)
                self.ui.imu_select.removeItem(index)
                index = self.ui.ft_select.findText(sensor)
                self.ui.ft_select.removeItem(index)