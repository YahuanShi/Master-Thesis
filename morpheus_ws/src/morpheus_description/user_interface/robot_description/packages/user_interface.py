import xml.etree.ElementTree as ET
import numpy as np
import os

class RobotDescriptionGUI:
    
    def __init__(self, ui, root, path):

        self.ui = ui
        self.root = root
        self.xacro_tag = '{http://www.ros.org/wiki/xacro}'
        self.package_path = path

    def read_xacro(self, current_link, macro):

        if current_link == "new link":
            self.robot_description_template_path = os.path.join(self.package_path, 'urdf/04_templates/robot_description.template.xacro')
            self.tree_template = ET.parse(self.robot_description_template_path)
            self.root_template = self.tree_template.getroot()
            self.ui.add_button.setEnabled(True)
            self.ui.remove_button.setEnabled(False)
            self.ui.preview_button.setEnabled(False)
            root = self.root_template
        else:
            self.ui.add_button.setEnabled(False)
            #self.ui.remove_button.setEnabled(True)
            self.ui.preview_button.setEnabled(True)
            root = self.root

        for self.element in root.iter(self.xacro_tag + macro):

            if self.element.attrib.get('name') == current_link:

                # Attributes
                self.name = self.element.attrib.get('name')
                self.mass = float(self.element.attrib.get('mass'))
                if macro == 'part':
                    self.parent = self.element.attrib.get('parent')
                    self.joint_type = self.element.attrib.get('type')
                    self.mimic = self.element.attrib.get('mimic')
                else:
                    self.empty = self.element.attrib.get('empty')
               
                # Link Elements
                self.link_viz_origin = self.element.find(self.xacro_tag + 'link_viz_origin')
                self.link_viz_x = float(self.link_viz_origin.get('xyz').split(' ')[0])
                self.link_viz_y = float(self.link_viz_origin.get('xyz').split(' ')[1])
                self.link_viz_z = float(self.link_viz_origin.get('xyz').split(' ')[2])
                self.link_viz_roll  = float(self.link_viz_origin.get('rpy').split(' ')[0])
                self.link_viz_pitch = float(self.link_viz_origin.get('rpy').split(' ')[1])
                self.link_viz_yaw   = float(self.link_viz_origin.get('rpy').split(' ')[2])

                self.link_col_origin = self.element.find(self.xacro_tag + 'link_col_origin')
                self.link_col_x = float(self.link_col_origin.get('xyz').split(' ')[0])
                self.link_col_y = float(self.link_col_origin.get('xyz').split(' ')[1])
                self.link_col_z = float(self.link_col_origin.get('xyz').split(' ')[2])
                self.link_col_roll  = float(self.link_col_origin.get('rpy').split(' ')[0])
                self.link_col_pitch = float(self.link_col_origin.get('rpy').split(' ')[1])
                self.link_col_yaw   = float(self.link_col_origin.get('rpy').split(' ')[2])

                self.link_viz_shape = self.element.find(self.xacro_tag + 'link_viz_shape')
                self.shape_type_viz = self.link_viz_shape[0].tag
                if self.shape_type_viz == 'mesh':
                    self.link_viz_mesh = self.link_viz_shape[0].get('filename')
                elif self.shape_type_viz == 'box':
                    self.link_viz_box_x = float(self.link_viz_shape[0].get('size').split(' ')[0])
                    self.link_viz_box_y = float(self.link_viz_shape[0].get('size').split(' ')[1])
                    self.link_viz_box_z = float(self.link_viz_shape[0].get('size').split(' ')[2])
                elif self.shape_type_viz == 'cylinder':
                    self.link_viz_cyl_l = float(self.link_viz_shape[0].get('length'))          
                    self.link_viz_cyl_r = float(self.link_viz_shape[0].get('radius'))
                elif self.shape_type_viz == 'sphere':
                    self.link_viz_sph_r = float(self.link_viz_shape[0].get('radius'))

                self.link_col_shape = self.element.find(self.xacro_tag + 'link_col_shape')
                self.shape_type_col = self.link_col_shape[0].tag
                if self.shape_type_col == 'mesh':
                    self.link_col_mesh = self.link_col_shape[0].get('filename')
                elif self.shape_type_col == 'box':
                    self.link_col_box_x = float(self.link_col_shape[0].get('size').split(' ')[0])
                    self.link_col_box_y = float(self.link_col_shape[0].get('size').split(' ')[1])
                    self.link_col_box_z = float(self.link_col_shape[0].get('size').split(' ')[2])
                elif self.shape_type_col == 'cylinder':
                    self.link_col_cyl_l = float(self.link_col_shape[0].get('length'))          
                    self.link_col_cyl_r = float(self.link_col_shape[0].get('radius'))
                elif self.shape_type_col == 'sphere':
                    self.link_col_sph_r = float(self.link_col_shape[0].get('radius'))

                self.link_color = self.element.find(self.xacro_tag + 'link_color')
                self.link_rgba = self.link_color.attrib.get('rgba').split()

                red = int(255*float(self.link_rgba[0]))
                green = int(255*float(self.link_rgba[1]))
                blue = int(255*float(self.link_rgba[2]))
                
                self.rgb = '#%02x%02x%02x' % (red, green, blue)

                self.link_center_of_mass = self.element.find(self.xacro_tag + 'link_center_of_mass')
                self.link_com_x = float(self.link_center_of_mass.get('xyz').split(' ')[0])
                self.link_com_y = float(self.link_center_of_mass.get('xyz').split(' ')[1])
                self.link_com_z = float(self.link_center_of_mass.get('xyz').split(' ')[2])

                self.link_inertia = self.element.find(self.xacro_tag + 'link_inertia')
                self.link_xx = float(self.link_inertia.get('ixx'))
                self.link_yy = float(self.link_inertia.get('iyy'))
                self.link_zz = float(self.link_inertia.get('izz'))
                self.link_xy = float(self.link_inertia.get('ixy'))
                self.link_xz = float(self.link_inertia.get('ixz'))
                self.link_yz = float(self.link_inertia.get('iyz'))

                # Joint Elements
                if macro == 'part':
                    self.joint_origin = self.element.find(self.xacro_tag + 'joint_origin')
                    self.joint_x = float(self.joint_origin.get('xyz').split(' ')[0])
                    self.joint_y = float(self.joint_origin.get('xyz').split(' ')[1])
                    self.joint_z = float(self.joint_origin.get('xyz').split(' ')[2])
                    self.joint_roll  = float(self.joint_origin.get('rpy').split(' ')[0])
                    self.joint_pitch = float(self.joint_origin.get('rpy').split(' ')[1])
                    self.joint_yaw   = float(self.joint_origin.get('rpy').split(' ')[2])

                    self.joint_axis_of_rotation = self.element.find(self.xacro_tag + 'joint_axis_of_rotation')
                    self.joint_aor_x = float(self.joint_axis_of_rotation.get('xyz').split(' ')[0])
                    self.joint_aor_y = float(self.joint_axis_of_rotation.get('xyz').split(' ')[1])
                    self.joint_aor_z = float(self.joint_axis_of_rotation.get('xyz').split(' ')[2])

                    self.joint_limits = self.element.find(self.xacro_tag + 'joint_limits')
                    self.joint_effort = float(self.joint_limits.get('effort'))
                    self.joint_velocity = float(self.joint_limits.get('velocity'))
                    self.joint_upper = float(self.joint_limits.get('upper'))
                    self.joint_lower = float(self.joint_limits.get('lower'))

                    self.joint_dynamics = self.element.find(self.xacro_tag + 'joint_dynamics')
                    self.joint_damping = float(self.joint_dynamics.get('damping'))
                    self.joint_friction = float(self.joint_dynamics.get('friction'))

                    self.joint_mimic = self.element.find(self.xacro_tag + 'joint_mimic')
                    self.joint_multiplier = float(self.joint_mimic.get('multiplier'))
                    self.joint_offset = float(self.joint_mimic.get('offset'))
                    self.joint_mimic_joint = self.joint_mimic.get('joint')

    def set_user_interface(self):
        
        self.ui.name_line.setText(self.name)
        self.ui.link_origin_x_col.setToolTip(str(self.name))
        self.ui.mass_line.setValue(self.mass)
        self.ui.link_origin_x_col.setToolTip(str(self.mass))
        if self.ui.comboBox.currentIndex() != 0:
            self.ui.comboBox_parent.setCurrentText(self.parent)
            self.ui.comboBox_parent.setToolTip(str(self.parent))
            self.ui.comboBox_joints.setCurrentText(self.joint_type)
            self.ui.checkBox_mimic.setChecked(int(self.mimic))
        else:
            self.ui.checkBox_base.setChecked(int(self.empty))

        # Parsing the values from the xacro file to the UI
        self.ui.link_origin_x_viz.setValue(self.link_viz_x)
        self.ui.link_origin_x_viz.setToolTip(str(self.link_viz_x))
        self.ui.link_origin_y_viz.setValue(self.link_viz_y)
        self.ui.link_origin_y_viz.setToolTip(str(self.link_viz_y))
        self.ui.link_origin_z_viz.setValue(self.link_viz_z)
        self.ui.link_origin_z_viz.setToolTip(str(self.link_viz_z))

        self.ui.link_origin_r_viz.setValue(np.rad2deg(self.link_viz_roll))
        self.ui.link_origin_r_viz.setToolTip(str(np.rad2deg(self.link_viz_roll)))
        self.ui.link_origin_p_viz.setValue(np.rad2deg(self.link_viz_pitch))
        self.ui.link_origin_p_viz.setToolTip(str(np.rad2deg(self.link_viz_pitch)))
        self.ui.link_origin_yaw_viz.setValue(np.rad2deg(self.link_viz_yaw))
        self.ui.link_origin_yaw_viz.setToolTip(str(np.rad2deg(self.link_viz_yaw)))
        ###
        if self.shape_type_viz == 'mesh':
            self.ui.radioButton_mesh_viz.setChecked(True)
            self.ui.viz_mesh_line.setText(self.link_viz_mesh)
            self.ui.viz_mesh_line.setToolTip(str(self.link_viz_mesh))
        elif self.shape_type_viz == 'box':
            self.ui.radioButton_box_viz.setChecked(True)
            self.ui.box_x_viz.setValue(self.link_viz_box_x)
            self.ui.box_x_viz.setToolTip(str(self.link_viz_box_x))
            self.ui.box_y_viz.setValue(self.link_viz_box_y)
            self.ui.box_y_viz.setToolTip(str(self.link_viz_box_y))
            self.ui.box_z_viz.setValue(self.link_viz_box_z)
            self.ui.box_z_viz.setToolTip(str(self.link_viz_box_z))
        elif self.shape_type_viz == 'cylinder':
            self.ui.radioButton_cylinder_viz.setChecked(True)
            self.ui.length_cyl_viz.setValue(self.link_viz_cyl_l)
            self.ui.length_cyl_viz.setToolTip(str(self.link_viz_cyl_l))            
            self.ui.radius_cyl_viz.setValue(self.link_viz_cyl_r)
            self.ui.radius_cyl_viz.setToolTip(str(self.link_viz_cyl_r))
        elif self.shape_type_viz == 'sphere':
            self.ui.radioButton_sphere_viz.setChecked(True)
            self.ui.radius_sph_viz.setValue(self.link_viz_sph_r)
            self.ui.radius_sph_viz.setToolTip(str(self.link_viz_sph_r))

        self.ui.toolButton_color.setStyleSheet("background-color : " + self.rgb)

        self.ui.link_origin_x_col.setValue(self.link_col_x)
        self.ui.link_origin_x_col.setToolTip(str(self.link_col_x))
        self.ui.link_origin_y_col.setValue(self.link_col_y)
        self.ui.link_origin_y_col.setToolTip(str(self.link_col_y))
        self.ui.link_origin_z_col.setValue(self.link_col_z)
        self.ui.link_origin_z_col.setToolTip(str(self.link_col_z))

        self.ui.link_origin_r_col.setValue(np.rad2deg(self.link_col_roll))
        self.ui.link_origin_r_col.setToolTip(str(np.rad2deg(self.link_col_roll)))
        self.ui.link_origin_p_col.setValue(np.rad2deg(self.link_col_pitch))
        self.ui.link_origin_p_col.setToolTip(str(np.rad2deg(self.link_col_pitch)))
        self.ui.link_origin_yaw_col.setValue(np.rad2deg(self.link_col_yaw))
        self.ui.link_origin_yaw_col.setToolTip(str(np.rad2deg(self.link_col_yaw)))
        ###
        if self.shape_type_col == 'mesh':
            self.ui.radioButton_mesh_col.setChecked(True)
            self.ui.col_mesh_line.setText(self.link_col_mesh)
            self.ui.col_mesh_line.setToolTip(str(self.link_col_mesh))
        elif self.shape_type_col == 'box':
            self.ui.radioButton_box_col.setChecked(True)
            self.ui.box_x_col.setValue(self.link_col_box_x)
            self.ui.box_x_col.setToolTip(str(self.link_col_box_x))
            self.ui.box_y_col.setValue(self.link_col_box_y)
            self.ui.box_y_col.setToolTip(str(self.link_col_box_y))
            self.ui.box_z_col.setValue(self.link_col_box_z)
            self.ui.box_z_col.setToolTip(str(self.link_col_box_z))
        elif self.shape_type_col == 'cylinder':
            self.ui.radioButton_cylinder_col.setChecked(True)
            self.ui.length_cyl_col.setValue(self.link_col_cyl_l)
            self.ui.length_cyl_col.setToolTip(str(self.link_col_cyl_l))            
            self.ui.radius_cyl_col.setValue(self.link_col_cyl_r)
            self.ui.radius_cyl_col.setToolTip(str(self.link_col_cyl_r))
        elif self.shape_type_col == 'sphere':
            self.ui.radioButton_sphere_col.setChecked(True)
            self.ui.radius_sph_col.setValue(self.link_col_sph_r)
            self.ui.radius_sph_col.setToolTip(str(self.link_col_sph_r))

        self.ui.link_cof_x.setValue(self.link_com_x)
        self.ui.link_cof_x.setToolTip(str(self.link_com_x))
        self.ui.link_cof_y.setValue(self.link_com_y)
        self.ui.link_cof_y.setToolTip(str(self.link_com_y))
        self.ui.link_cof_z.setValue(self.link_com_z)
        self.ui.link_cof_z.setToolTip(str(self.link_com_z))

        self.ui.link_moi_xx.setValue(self.link_xx)
        self.ui.link_moi_xx.setToolTip(str(self.link_xx))
        self.ui.link_moi_yy.setValue(self.link_yy)
        self.ui.link_moi_yy.setToolTip(str(self.link_yy))
        self.ui.link_moi_zz.setValue(self.link_zz)
        self.ui.link_moi_zz.setToolTip(str(self.link_zz))

        self.ui.link_moi_xy.setValue(self.link_xy)
        self.ui.link_moi_xy.setToolTip(str(self.link_xy))
        self.ui.link_moi_xz.setValue(self.link_xz)
        self.ui.link_moi_xz.setToolTip(str(self.link_xz))
        self.ui.link_moi_yz.setValue(self.link_yz)
        self.ui.link_moi_yz.setToolTip(str(self.link_yz))

        #---------------------------------------------------------------------

        if self.ui.comboBox.currentIndex() != 0:
            self.ui.joint_origin_x.setValue(self.joint_x)
            self.ui.joint_origin_x.setToolTip(str(self.joint_x))
            self.ui.joint_origin_y.setValue(self.joint_y)
            self.ui.joint_origin_y.setToolTip(str(self.joint_y))
            self.ui.joint_origin_z.setValue(self.joint_z)
            self.ui.joint_origin_z.setToolTip(str(self.joint_z))

            self.ui.joint_origin_r.setValue(np.rad2deg(self.joint_roll))
            self.ui.joint_origin_r.setToolTip(str(np.rad2deg(self.joint_roll)))
            self.ui.joint_origin_p.setValue(np.rad2deg(self.joint_pitch))
            self.ui.joint_origin_p.setToolTip(str(np.rad2deg(self.joint_pitch)))
            self.ui.joint_origin_yaw.setValue(np.rad2deg(self.joint_yaw))
            self.ui.joint_origin_yaw.setToolTip(str(np.rad2deg(self.joint_yaw)))
            
            self.ui.joint_aor_x.setValue(self.joint_aor_x)
            self.ui.joint_aor_x.setToolTip(str(self.joint_aor_x))
            self.ui.joint_aor_y.setValue(self.joint_aor_y)
            self.ui.joint_aor_y.setToolTip(str(self.joint_aor_y))
            self.ui.joint_aor_z.setValue(self.joint_aor_z)
            self.ui.joint_aor_z.setToolTip(str(self.joint_aor_z))

            self.ui.joint_limit_effort.setValue(self.joint_effort)
            self.ui.joint_limit_effort.setToolTip(str(self.joint_effort))
            self.ui.joint_limit_vel.setValue(self.joint_velocity)
            self.ui.joint_limit_vel.setToolTip(str(self.joint_velocity))
            self.ui.joint_limit_lower.setValue(np.rad2deg(self.joint_lower))
            self.ui.joint_limit_lower.setToolTip(str(self.joint_lower))
            self.ui.joint_limit_upper.setValue(np.rad2deg(self.joint_upper))
            self.ui.joint_limit_upper.setToolTip(str(self.joint_upper))

            self.ui.joint_damping.setValue(self.joint_damping)
            self.ui.joint_damping.setToolTip(str(self.joint_damping))
            self.ui.joint_friction.setValue(self.joint_friction)
            self.ui.joint_friction.setToolTip(str(self.joint_friction))

            self.ui.joint_mimic_mult.setValue(self.joint_multiplier)
            self.ui.joint_mimic_mult.setToolTip(str(self.joint_multiplier))
            self.ui.joint_mimic_offset.setValue(self.joint_offset)
            self.ui.joint_mimic_offset.setToolTip(str(self.joint_offset))
            self.ui.comboBox_mimic.setCurrentText(self.joint_mimic_joint)
            self.ui.comboBox_mimic.setToolTip(str(self.joint_mimic_joint))
