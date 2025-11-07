import os

from PyQt6.QtWidgets import QFileDialog

class URDFImporterFunctions:
    def __init__(self, ui):
        self.ui = ui

    def select_urdf_file(self):
        file_filter = 'URDF File (*.urdf *.URDF)'
        response = QFileDialog.getOpenFileName(
            parent=self.ui,
            caption='Select an URDF file',
            directory=os.path.expanduser('~'),
            filter=file_filter,
            initialFilter=file_filter
        )
        return response[0]

    def select_urdf_package(self):
        directory = QFileDialog.getExistingDirectory(
            self.ui,
            caption='Select the package where the urdf is located',
            directory=os.path.expanduser('~')
        )
        return directory
    
    def get_urdf_link_names(self, root):

        link_names = []

        for element in root.iter('link'):

            name = element.attrib.get('name')
            # name = name.replace('_link', '')
            link_names.append(name)

        return link_names