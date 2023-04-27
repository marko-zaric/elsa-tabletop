import xml.etree.ElementTree as ET
import rospkg as rp
import rospy
from elsa_object_database.srv import RegisteredObjects
import random


class SDFmodifier:
    def __init__(self):
        rospy.loginfo("Waiting for get_registered_obj_service")
        rospy.wait_for_service("get_registered_obj_service")
        self.registered_objs = []
        try:
            get_registered_objs = rospy.ServiceProxy("get_registered_obj_service", RegisteredObjects)
            response = get_registered_objs()
            for obj in response.registered_objects:
                self.registered_objs.append([obj.object_name, obj.gazebo_color, obj.r_value, obj.g_value, obj.b_value, obj.alpha_value])
        except rospy.ServiceException as e:
            print("Registered Objects Service failed %s", e)

    def set_color(self, sdf_xml, color):
        stringroot = ET.fromstring(sdf_xml)
        for material in stringroot.iter('material'):
            ambient = material.find('ambient')
            script = material.find('script')

            #remove present color tags
            if ambient is not None:
                material.remove(ambient)
            if script is not None:
                material.remove(script)

            color_info = None
            for reg_obj in self.registered_objs:
                if color == reg_obj[0]:
                    color_info = reg_obj
                    break
            
            color_tag = self.build_new_color_tag(color_info)
            material.insert(0, color_tag)

        return ET.tostring(stringroot, encoding='unicode', method='xml')

    def set_size(self, sdf_xml, size):
        x = size[0]
        y = size[1]
        z = size[2]
        
        stringroot = ET.fromstring(sdf_xml)
        model_name = stringroot.find('./model').attrib['name']

        for geometry in stringroot.iter('geometry'):
            box = geometry.find('box')
            sphere = geometry.find('sphere')
            cylinder = geometry.find('cylinder')

            if model_name == 'rectangle':
                size = box.find('size')
                size.text = str(x) + ' ' + str(y) + ' ' + str(z)
            elif model_name == 'cube':
                size = box.find('size')
                size.text = str(x) + ' ' + str(y) + ' ' + str(z)
            elif model_name == 'sphere':
                radius = sphere.find('radius')
                r = x/2
                radius.text = str(r)
            elif model_name == 'cylinder':
                r = x/2
                l = z
                radius = cylinder.find('radius')
                length = cylinder.find('length')
                radius.text = str(r)
                length.text = str(l)
                

        return ET.tostring(stringroot, encoding='unicode', method='xml')

    def randomize_color(self, sdf_xml):
        stringroot = ET.fromstring(sdf_xml)
        for material in stringroot.iter('material'):
            ambient = material.find('ambient')
            script = material.find('script')

            #remove present color tags
            if ambient is not None:
                material.remove(ambient)
            if script is not None:
                material.remove(script)
            
            # Select random color
            color = random.choice(self.registered_objs)
            self.registered_objs.remove(color)
            color_name = color[0]

            color_tag = self.build_new_color_tag(color)
            material.insert(0, color_tag)

        return color_name, ET.tostring(stringroot, encoding='unicode', method='xml')
        

    def randomize_size(self, sdf_xml):
        stringroot = ET.fromstring(sdf_xml)
        model_name = stringroot.find('./model').attrib['name']

        min_size = 4
        if model_name == 'rectangle':
            x = (random.random() + random.randint(min_size, 8)) / 100
            y = (random.random() + random.randint(min_size, 8)) / 100
            z = (random.random() + random.randint(min_size, 8)) / 100
        elif model_name == 'cube':
            x = (random.random() + random.randint(min_size, 8)) / 100
        elif model_name == 'sphere':
            r = (random.random() + random.randint(min_size,7)) / 100
        elif model_name == 'cylinder':
            r = (random.random() + random.randint(min_size,7)) / 100
            l = (random.random() + random.randint(min_size,10)) / 100
        
        for geometry in stringroot.iter('geometry'):
            box = geometry.find('box')
            sphere = geometry.find('sphere')
            cylinder = geometry.find('cylinder')

            GT_bounding_box = []
            is_round = False
            #randomize size
            if model_name == 'rectangle':
                size = box.find('size')
                size.text = str(x) + ' ' + str(y) + ' ' + str(z)
                GT_bounding_box.append(x)
                GT_bounding_box.append(y)
                GT_bounding_box.append(z)
            elif model_name == 'cube':
                size = box.find('size')
                size.text = str(x) + ' ' + str(x) + ' ' + str(x)
                GT_bounding_box.append(x)
                GT_bounding_box.append(x)
                GT_bounding_box.append(x)
            elif model_name == 'sphere':
                radius = sphere.find('radius')
                radius.text = str(r)
                GT_bounding_box.append(r*2)
                GT_bounding_box.append(r*2)
                GT_bounding_box.append(r*2)
                is_round = True
            elif model_name == 'cylinder':
                radius = cylinder.find('radius')
                length = cylinder.find('length')
                radius.text = str(r)
                length.text = str(l)
                GT_bounding_box.append(r*2)
                GT_bounding_box.append(r*2)
                GT_bounding_box.append(l)
                is_round = True
                

        return GT_bounding_box, ET.tostring(stringroot, encoding='unicode', method='xml'), is_round



    def build_new_color_tag(self, color_info):
        # Distinguish between gazebo built in color and ambient rgba color
        return_color_tag = None
        if color_info[1] == 'ambient color':
            return_color_tag = ET.Element('ambient')
            return_color_tag.text = str(color_info[2]) + ' ' + str(color_info[3]) + ' ' + str(color_info[4]) + ' ' + str(color_info[5])
        else:
            return_color_tag = ET.Element('script')
            uri_tag = ET.SubElement(return_color_tag, 'uri')
            uri_tag.text = 'file://media/materials/scripts/gazebo.material'
            name_tag = ET.SubElement(return_color_tag, 'name')
            name_tag.text = color_info[1]
        return return_color_tag
