import xml.etree.ElementTree as ET

class GazeboFunctions:
    def __init__(self, path):
        self.gazebo_sensors_path = path

    def get_sensor_names(self, root):

        self.camera_list = []
        self.lidar_list = []
        self.imu_list = []
        self.ft_list = []

        for self.element in root.iter('gazebo'):    
            if self.element.attrib.get('reference') is not None:
                sensor_element = self.element.find('sensor')
                if 'camera' in sensor_element.attrib.get('type'):
                    name = sensor_element.attrib.get('name')
                    reference = self.element.attrib.get('reference')
                    camera = str(name + ' | ' + reference)
                    self.camera_list.append(camera)
                elif 'lidar' in sensor_element.attrib.get('type'):
                    name = sensor_element.attrib.get('name')
                    reference = self.element.attrib.get('reference')
                    lidar = str(name + ' | ' + reference)
                    self.lidar_list.append(lidar)
                elif 'imu' in sensor_element.attrib.get('type'):
                    name = sensor_element.attrib.get('name')
                    reference = self.element.attrib.get('reference')
                    imu = str(name + ' | ' + reference)
                    self.imu_list.append(imu)
                elif 'force_torque' in sensor_element.attrib.get('type'):
                    name = sensor_element.attrib.get('name')
                    reference = self.element.attrib.get('reference')
                    ft = str(name + ' | ' + reference)
                    self.ft_list.append(ft)
            
        return self.camera_list, self.lidar_list, self.imu_list, self.ft_list

    def remove_sensor(self, root, name, reference):
        self.element = root.find(".//sensor[@name='" + name + "']/..[@reference='" + reference + "']")
        root.remove(self.element)

    def write_file(self, tree):

        # self.root_gazebo.tail = "\n"
        ET.indent(tree,'  ', level=0)
        ET.register_namespace("xacro", 'http://www.ros.org/wiki/xacro')
        tree.write(self.gazebo_sensors_path, 'UTF-8')

        return self.element