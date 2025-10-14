import numpy as np

class URDFImporterGUI:

    def __init__(self, ui, robot_description):
        self.ui = ui
        self.robot_description = robot_description
        self.robot_description_xacro = self.robot_description.robot_description_xacro

    def read_urdf(self, link_name, root):

        for element in root.iter('link'):

            if element.attrib.get('name') == link_name:

                self.robot_description_xacro.name = element.attrib.get('name')
                # TODO: Für einzeilige Base Links
                # inertial tag
                inertial = element.find('inertial')
                if inertial is not None:
                    self.robot_description_xacro.empty = '0'

                self.robot_description_xacro.link_com = inertial.find('origin').attrib.get('xyz')
                self.robot_description_xacro.mass = inertial.find('mass').attrib.get('value')
                self.robot_description_xacro.link_xx = inertial.find('inertia').attrib.get('ixx')
                self.robot_description_xacro.link_xy = inertial.find('inertia').attrib.get('ixy')
                self.robot_description_xacro.link_xz = inertial.find('inertia').attrib.get('ixz')
                self.robot_description_xacro.link_yy = inertial.find('inertia').attrib.get('iyy')
                self.robot_description_xacro.link_yz = inertial.find('inertia').attrib.get('iyz')
                self.robot_description_xacro.link_zz = inertial.find('inertia').attrib.get('izz')

                # visual tag
                visual = element.find('visual')
                self.robot_description_xacro.link_viz_xyz = visual.find('origin').attrib.get('xyz')
                self.robot_description_xacro.link_viz_rpy = visual.find('origin').attrib.get('rpy')

                geometry = visual.find('geometry')
                self.robot_description_xacro.shape_type_viz = geometry[0].tag

                if self.robot_description_xacro.shape_type_viz == 'mesh':
                    self.robot_description_xacro.new_tag = 'mesh'
                    self.robot_description_xacro.new_attr_name = 'filename'
                    self.robot_description_xacro.link_viz_mesh = geometry.find('mesh').attrib.get('filename')
                elif self.robot_description_xacro.shape_type_viz == 'box':
                    self.robot_description_xacro.new_tag = 'box'
                    self.robot_description_xacro.new_attr_name = 'size'
                    self.robot_description_xacro.link_viz_box = geometry.find('box').attrib.get('size')
                elif self.robot_description_xacro.shape_type_viz == 'cylinder':
                    self.robot_description_xacro.new_tag = 'cylinder'
                    self.robot_description_xacro.new_attr_name_r = 'radius'
                    self.robot_description_xacro.new_attr_name_l = 'length'
                    self.robot_description_xacro.link_viz_cyl_l = geometry.find('cylinder').attrib.get('length')
                    self.robot_description_xacro.link_viz_cyl_r = geometry.find('cylinder').attrib.get('radius')
                elif self.robot_description_xacro.shape_type_viz == 'sphere':
                    self.robot_description_xacro.new_tag = 'sphere'
                    self.robot_description_xacro.new_attr_name = 'radius'
                    self.robot_description_xacro.link_viz_sph_r = geometry.find('sphere').attrib.get('radius')
        
                # material tag 
                material = visual.find('material')
                if material is not None:
                    color = material.find('color')
                    if color is not None:
                        self.robot_description_xacro.link_rgba = color.attrib.get('rgba').split()

                        red = self.robot_description_xacro.link_rgba[0]
                        green = self.robot_description_xacro.link_rgba[1]
                        blue = self.robot_description_xacro.link_rgba[2]
                        
                        self.robot_description_xacro.rgb = str(red) + " " + str(green) + " " + str(blue) + " 1.0"
                    else:
                        self.robot_description_xacro.rgb = "0.6 0.6 0.6"
                else:
                    self.robot_description_xacro.rgb = "0.6 0.6 0.6"

                # collision tag
                collision = element.find('collision')
                if collision is not None:
                    origin = collision.find('origin')
                    if origin is not None:
                        self.robot_description_xacro.link_col_xyz = origin.attrib.get('xyz')
                        self.robot_description_xacro.link_col_rpy = origin.attrib.get('rpy')
                    else:
                        self.robot_description_xacro.link_col_xyz = '0.0 0.0 0.0'
                        self.robot_description_xacro.link_col_rpy = '0.0 0.0 0.0'

                    geometry = collision.find('geometry')
                    self.robot_description_xacro.shape_type_col = geometry[0].tag

                    if self.robot_description_xacro.shape_type_col == 'box':
                        self.robot_description_xacro.new_tag_col = 'box'
                        self.robot_description_xacro.new_attr_name_col = 'size'
                        self.robot_description_xacro.link_col_box = geometry.find('box').attrib.get('size')
                    elif self.robot_description_xacro.shape_type_col == 'sphere':
                        self.robot_description_xacro.new_tag_col = 'sphere'
                        self.robot_description_xacro.new_attr_name_col = 'radius'
                        self.robot_description_xacro.link_col_sph_r = geometry.find('sphere').attrib.get('radius')
                    elif self.robot_description_xacro.shape_type_col == 'cylinder':
                        self.robot_description_xacro.new_tag_col = 'cylinder'
                        self.robot_description_xacro.new_attr_name_r_col = 'radius'
                        self.robot_description_xacro.new_attr_name_l_col = 'length'
                        self.robot_description_xacro.link_col_cyl_r = geometry.find('cylinder').attrib.get('radius')
                        self.robot_description_xacro.link_col_cyl_l = geometry.find('cylinder').attrib.get('length')
                    elif self.robot_description_xacro.shape_type_col == 'mesh':
                        self.robot_description_xacro.new_tag_col = 'mesh'
                        self.robot_description_xacro.new_attr_name_col = 'filename'
                        self.robot_description_xacro.link_col_mesh = geometry.find('mesh').attrib.get('filename')
                else: 
                    self.robot_description_xacro.link_col_xyz = '0.0 0.0 0.0'
                    self.robot_description_xacro.link_col_rpy = '0.0 0.0 0.0'
                    self.robot_description_xacro.shape_type_col = 'box'
                    self.robot_description_xacro.new_tag_col = 'box'
                    self.robot_description_xacro.new_attr_name_col = 'size'
                    self.robot_description_xacro.link_col_box = '1 1 1'

                self.robot_description_xacro.joint = root.find('.//child[@link="' + link_name + '"]/..')
                if self.robot_description_xacro.joint is not None:    
                    # joint type
                    self.robot_description_xacro.joint_type = self.robot_description_xacro.joint.attrib.get('type')
                    # origin tag
                    origin = self.robot_description_xacro.joint.find('origin')
                    self.robot_description_xacro.joint_xyz = origin.attrib.get('xyz')
                    self.robot_description_xacro.joint_rpy = origin.attrib.get('rpy')
                    # parent tag
                    parent = self.robot_description_xacro.joint.find('parent')
                    self.robot_description_xacro.parent = parent.attrib.get('link').replace('_link','')
                    # child tag
                    child = self.robot_description_xacro.joint.find('child')
                    self.robot_description_xacro.child = child.attrib.get('link')
                    # axis tag
                    axis = self.robot_description_xacro.joint.find('axis')
                    if axis is not None:
                        self.robot_description_xacro.joint_aor = axis.attrib.get('xyz')
                    else:
                        self.robot_description_xacro.joint_aor = "1.0 0.0 0.0"
                    # limit tag
                    limits = self.robot_description_xacro.joint.find('limit')
                    if limits is not None:
                        self.robot_description_xacro.joint_lower = limits.attrib.get('lower')
                        self.robot_description_xacro.joint_upper = limits.attrib.get('upper')
                        if self.robot_description_xacro.joint_lower is None or self.robot_description_xacro.joint_upper is None:
                            self.robot_description_xacro.joint_lower = "0.0"
                            self.robot_description_xacro.joint_upper = "0.0"
                        self.robot_description_xacro.joint_effort = limits.attrib.get('effort')
                        self.robot_description_xacro.joint_velocity = limits.attrib.get('velocity')
                    else:
                        self.robot_description_xacro.joint_lower = "0.0"
                        self.robot_description_xacro.joint_upper = "0.0"
                        self.robot_description_xacro.joint_effort = "0.0"
                        self.robot_description_xacro.joint_velocity = "0.0"
                    # dynamics tag
                    dynamics = self.robot_description_xacro.joint.find('dynamics')
                    if dynamics is not None:
                        self.robot_description_xacro.joint_damping = dynamics.attrib.get('damping')
                        if self.robot_description_xacro.joint_damping is None:
                            self.robot_description_xacro.joint_damping = '0.0'
                        self.robot_description_xacro.joint_friction = dynamics.attrib.get('friction')
                        if self.robot_description_xacro.joint_friction is None:
                            self.robot_description_xacro.joint_friction = '0.0'

                    else:
                        self.robot_description_xacro.joint_damping = '0.0'
                        self.robot_description_xacro.joint_friction = '0.0'
                    # mimic tag
                    mimic = self.robot_description_xacro.joint.find('mimic')
                    if mimic is not None:
                        self.robot_description_xacro.mimic = '1'
                        self.robot_description_xacro.joint_mimic_joint = mimic.attrib.get('joint').replace('_joint','')
                        self.robot_description_xacro.joint_multiplier = mimic.attrib.get('multiplier')
                        self.robot_description_xacro.joint_offset = mimic.attrib.get('offset')
                        self.check_mimic_box = True
                    else:
                        self.robot_description_xacro.mimic = '0' 
                        self.robot_description_xacro.joint_mimic_joint = 'none'
                        self.robot_description_xacro.joint_multiplier = '0.0'
                        self.robot_description_xacro.joint_offset = '0.0'
                        self.check_mimic_box = False

    def set_user_interface(self, file_name, package_name):

        self.urdf_file = file_name
        self.path_pkg = package_name

        self.ui.lineEdit_filename.setText(self.urdf_file)
        self.ui.lineEdit_package_path.setText(self.path_pkg)

        self.ui.mass_line_import.setText(self.robot_description_xacro.mass)

        # if self.joint is not None:  
        #     self.ui.joint_type_line_import.setText(self.joint_type)
        #     self.ui.parent_line_import.setText(self.parent)
        #     self.ui.checkBox_mimic_import.setChecked(int(self.mimic))
        # else: # s.o. für base link
        #     self.ui.joint_type_line_import.setText('none')
        #     self.ui.parent_line_import.setText('none')
        #     self.ui.checkBox_mimic_import.setChecked(False)

        if self.ui.comboBox_import.currentIndex() != 0:
            self.ui.joint_type_line_import.setText(self.robot_description_xacro.joint_type)
            self.ui.parent_line_import.setText(self.robot_description_xacro.parent)
            self.ui.checkBox_mimic_import.setChecked(int(self.robot_description_xacro.mimic))
        else:
            self.ui.checkBox_base_import.setChecked(int(self.robot_description_xacro.empty))

        # Parsing the values from the xacro file to the UI
        self.ui.link_origin_x_import.setValue(float(self.robot_description_xacro.link_viz_xyz.split(' ')[0]))
        self.ui.link_origin_y_import.setValue(float(self.robot_description_xacro.link_viz_xyz.split(' ')[1]))
        self.ui.link_origin_z_import.setValue(float(self.robot_description_xacro.link_viz_xyz.split(' ')[2]))

        self.ui.link_origin_r_import.setValue(np.rad2deg(float(self.robot_description_xacro.link_viz_rpy.split(' ')[0])))
        self.ui.link_origin_p_import.setValue(np.rad2deg(float(self.robot_description_xacro.link_viz_rpy.split(' ')[1])))
        self.ui.link_origin_yaw_import.setValue(np.rad2deg(float(self.robot_description_xacro.link_viz_rpy.split(' ')[2])))

        if self.robot_description_xacro.shape_type_viz == 'mesh':
            self.ui.radioButton_mesh_viz_2.setChecked(True)
            self.ui.viz_mesh_line_2.setText(self.robot_description_xacro.link_viz_mesh)
        elif self.robot_description_xacro.shape_type_viz == 'box':
            self.ui.radioButton_box_viz_2.setChecked(True)
            self.ui.box_x_viz_2.setValue(float(self.robot_description_xacro.link_viz_box.split(' ')[0]))
            self.ui.box_y_viz_2.setValue(float(self.robot_description_xacro.link_viz_box.split(' ')[1]))
            self.ui.box_z_viz_2.setValue(float(self.robot_description_xacro.link_viz_box.split(' ')[2]))
        elif self.robot_description_xacro.shape_type_viz == 'cylinder':
            self.ui.radioButton_cylinder_viz_2.setChecked(True)
            self.ui.length_cyl_viz_2.setValue(float(self.robot_description_xacro.link_viz_cyl_l))            
            self.ui.radius_cyl_viz_2.setValue(float(self.robot_description_xacro.link_viz_cyl_r))
        elif self.robot_description_xacro.shape_type_viz == 'sphere':
            self.ui.radioButton_sphere_viz_2.setChecked(True)
            self.ui.radius_sph_viz_2.setValue(float(self.robot_description_xacro.link_viz_sph_r))

        if self.robot_description_xacro.shape_type_col == 'mesh':
            self.ui.radioButton_mesh_col_2.setChecked(True)
            self.ui.col_mesh_line_2.setText(self.robot_description_xacro.link_col_mesh)
        elif self.robot_description_xacro.shape_type_col == 'box':
            self.ui.radioButton_box_col_2.setChecked(True)
            self.ui.box_x_col_2.setValue(float(self.robot_description_xacro.link_col_box.split(' ')[0]))
            self.ui.box_y_col_2.setValue(float(self.robot_description_xacro.link_col_box.split(' ')[1]))
            self.ui.box_z_col_2.setValue(float(self.robot_description_xacro.link_col_box.split(' ')[2]))
        elif self.robot_description_xacro.shape_type_col == 'cylinder':
            self.ui.radioButton_cylinder_col_2.setChecked(True)
            self.ui.length_cyl_col_2.setValue(float(self.robot_description_xacro.link_col_cyl_l))            
            self.ui.radius_cyl_col_2.setValue(float(self.robot_description_xacro.link_col_cyl_r))
        elif self.robot_description_xacro.shape_type_col == 'sphere':
            self.ui.radioButton_sphere_col_2.setChecked(True)
            self.ui.radius_sph_col_2.setValue(float(self.robot_description_xacro.link_col_sph_r))

        self.ui.link_cof_x_import.setValue(float(self.robot_description_xacro.link_com.split(' ')[0]))
        self.ui.link_cof_y_import.setValue(float(self.robot_description_xacro.link_com.split(' ')[1]))
        self.ui.link_cof_z_import.setValue(float(self.robot_description_xacro.link_com.split(' ')[2]))

        self.ui.link_moi_xx_import.setValue(float(self.robot_description_xacro.link_xx))
        self.ui.link_moi_yy_import.setValue(float(self.robot_description_xacro.link_yy))
        self.ui.link_moi_zz_import.setValue(float(self.robot_description_xacro.link_zz))

        self.ui.link_moi_xy_import.setValue(float(self.robot_description_xacro.link_xy))
        self.ui.link_moi_xz_import.setValue(float(self.robot_description_xacro.link_xz))
        self.ui.link_moi_yz_import.setValue(float(self.robot_description_xacro.link_yz))

        #---------------------------------------------------------------------

        if self.robot_description_xacro.joint is not None:

            self.ui.joint_origin_x_import.setValue(float(self.robot_description_xacro.joint_xyz.split(' ')[0]))
            self.ui.joint_origin_y_import.setValue(float(self.robot_description_xacro.joint_xyz.split(' ')[1]))
            self.ui.joint_origin_z_import.setValue(float(self.robot_description_xacro.joint_xyz.split(' ')[2]))

            self.ui.joint_origin_r_import.setValue(np.rad2deg(float(self.robot_description_xacro.joint_rpy.split(' ')[0])))
            self.ui.joint_origin_p_import.setValue(np.rad2deg(float(self.robot_description_xacro.joint_rpy.split(' ')[1])))
            self.ui.joint_origin_yaw_import.setValue(np.rad2deg(float(self.robot_description_xacro.joint_rpy.split(' ')[2])))
            
            self.ui.joint_aor_x_import.setValue(float(self.robot_description_xacro.joint_aor.split(' ')[0]))
            self.ui.joint_aor_y_import.setValue(float(self.robot_description_xacro.joint_aor.split(' ')[1]))
            self.ui.joint_aor_z_import.setValue(float(self.robot_description_xacro.joint_aor.split(' ')[2]))

            self.ui.joint_limit_effort_import.setValue(float(self.robot_description_xacro.joint_effort))
            self.ui.joint_limit_vel_import.setValue(float(self.robot_description_xacro.joint_velocity))
            self.ui.joint_limit_lower_import.setValue(float(self.robot_description_xacro.joint_lower))
            self.ui.joint_limit_upper_import.setValue(float(self.robot_description_xacro.joint_upper))

            self.ui.joint_damping_import.setValue(float(self.robot_description_xacro.joint_damping))
            self.ui.joint_friction_import.setValue(float(self.robot_description_xacro.joint_friction))

            self.ui.joint_mimic_mult_import.setValue(float(self.robot_description_xacro.joint_multiplier))
            self.ui.joint_mimic_offset_import.setValue(float(self.robot_description_xacro.joint_offset))
            self.ui.mimic_joint_import.setText(self.robot_description_xacro.joint_mimic_joint)
            self.ui.checkBox_mimic_import.setChecked(self.check_mimic_box)