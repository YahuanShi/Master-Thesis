import subprocess, xacro, os, signal

class ROS2Tools: 

    def __init__(self, path):

        self.package_path = path
        self.xacro_morpheus = os.path.join(self.package_path, 'urdf/01_robot_description/robot.xacro')
        self.urdf_morpheus = os.path.join(self.package_path, 'urdf/robot.urdf')

        # self.xacro_morpheus = os.path.join(self.package_path, 'urdf/robot.xacro')
        # self.urdf_morpheus = os.path.join(self.package_path, 'urdf/robot.urdf')

        self.xacro_to_urdf()

        self.is_running = False
        self.process_robot_state = None
        self.process_joint_state = None
        self.process_rviz = None
        self.process_joint_state_gui = None

    def xacro_to_urdf(self):
    
                # 1. 存储原始工作目录
        original_cwd = os.getcwd()

        # 2. 确定文件目录和文件名
        xacro_dir = os.path.dirname(self.xacro_morpheus)
        xacro_filename = os.path.basename(self.xacro_morpheus)
        
        try:
            # 3. 临时切换到 xacro 文件所在的目录
            os.chdir(xacro_dir)

            # 4. 使用相对文件名调用 xacro.process_file
            # 此时 xacro 只需查找 'robot.xacro'
            doc = xacro.process_file(xacro_filename)

            robot_desc = doc.toprettyxml(indent='  ')
            
            # 5. 写入 URDF 文件（使用绝对路径，确保写在正确的位置）
            with open(self.urdf_morpheus, 'w') as f_w:
                f_w.write(robot_desc)
                f_w.close()
                
        except Exception as e:
            # 发生任何异常，都需要确保恢复 CWD
            os.chdir(original_cwd)
            print(f"Xacro 处理错误。目录: {xacro_dir}")
            raise e
            
        finally:
            # 6. 无论是否发生异常，都必须恢复到原始工作目录
            os.chdir(original_cwd)

    def start_joint_GUI(self): # ToDo: Stop GUI and set button label to: stop GUI

        cmd_joint_state = ['ros2', 'run', 'joint_state_publisher_gui','joint_state_publisher_gui',
                           '--ros-args', '-r', '__node:=joint_state_gui']
        self.process_joint_state_gui = subprocess.Popen(
            cmd_joint_state,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid 
            ) 

        if self.process_joint_state.poll() is None:
            print(f"Started Joint State Publisher with PID: {self.process_joint_state.pid}")
        else:
            error = self.process_joint_state.stderr.read().decode()
            print(f"Failed to start: {error}")

    def start_robot_description(self):

        cmd_robot_state = ['ros2', 'run', 'robot_state_publisher','robot_state_publisher', self.urdf_morpheus]
        
        self.process_robot_state = subprocess.Popen(
            cmd_robot_state,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid 
        )

        if self.process_robot_state.poll() is None:
            print(f"Started Robot State Publisher with PID: {self.process_robot_state.pid}")
        else:
            error = self.process_robot_state.stderr.read().decode()
            print(f"Failed to start: {error}")

    def start_rviz(self):

        config_file = '/home/oscar/Dokumente/Masterarbeit/morpheus_ws/src/morpheus_description/user_interface/default.rviz' # TODO: generic
        cmd_rviz = ['ros2', 'run', 'rviz2', 'rviz2', '-d', config_file]
        self.process_rviz = subprocess.Popen(
            cmd_rviz,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid 
        )

        if self.process_rviz.poll() is None:
            print(f"Started Rviz 2 with PID: {self.process_rviz.pid}")
        else:
            error = self.process_rviz.stderr.read().decode()
            print(f"Failed to start: {error}")

    def start_joint_state(self):

        cmd_joint_state = ['ros2', 'run', 'joint_state_publisher','joint_state_publisher']

        self.process_joint_state = subprocess.Popen(
            cmd_joint_state,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Create new process group
        )

        if self.process_joint_state.poll() is None:
            print(f"Started Joint State Publisher with PID: {self.process_joint_state.pid}")
        else:
            error = self.process_joint_state.stderr.read().decode()
            print(f"Failed to start: {error}")

    def start_processes(self):
        self.start_joint_state()
        self.start_robot_description()
        self.start_rviz()
        self.is_running = True

    def stop_processes(self):

        if self.is_running:
            
            os.killpg(self.process_robot_state.pid, signal.SIGINT)
            os.killpg(self.process_joint_state.pid, signal.SIGINT)
            os.killpg(self.process_rviz.pid, signal.SIGINT)

            if self.process_joint_state_gui is not None:
                os.killpg(self.process_joint_state_gui.pid, signal.SIGINT)

            print('Processes ended...')

            self.process_robot_state = None
            self.process_joint_state = None
            self.process_rviz = None
            self.is_running = False

    def update_robot_description(self):

        os.killpg(os.getpgid(self.process_robot_state.pid), signal.SIGINT)
        os.killpg(os.getpgid(self.process_joint_state.pid), signal.SIGINT)

        cmd_robot_state = ['ros2', 'run', 'robot_state_publisher','robot_state_publisher', self.urdf_morpheus]
        self.process_robot_state = subprocess.Popen(
            cmd_robot_state,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  
        )

        cmd_joint_state = ['ros2', 'run', 'joint_state_publisher','joint_state_publisher']
        self.process_joint_state = subprocess.Popen(
            cmd_joint_state,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Create new process group
        )
