import socket
import xml.etree.ElementTree as ET
import os
import re
import yaml
import zmq
import argparse
import logging
from logging.handlers import TimedRotatingFileHandler

base_path = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser(description='Serial number of pose')


logfile = r'D:\KUKA\KUKA_Controller\logfile\KUKA_Robot.log'
logger = logging.getLogger(__name__)

# 0:00 is the update point every day, a file is generated every day, and 30 logs are saved
handler = TimedRotatingFileHandler(filename=logfile, when="MIDNIGHT", interval=1, backupCount=30)
handler.suffix = "%Y-%m-%d.log"
handler.extMatch = re.compile(r"^\d{4}-\d{2}-\d{2}.log$")
logging.basicConfig(filename=logfile, level = logging.INFO, format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
# file logger
logger.addHandler(handler)
# console logger
console = logging.StreamHandler()
console.setLevel(logging.INFO)
logger.addHandler(console)
# shutdown logger
# logging.shutdown()

Robot_IP = '192.168.1.106'
Robot_PORT = 54600

zmq_IP = '192.168.1.20'
zmq_PORT = 5555

class Robot_connect:
    def __init__(self):
        logger.info("Robot connecting......")
        try:
            self.client = self.tcp_connect()
            logger.info("Robot connect successfully")
        except Exception as e:
            logger.info("Robot connect error:",e)

    def tcp_connect(self):
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect((Robot_IP, Robot_PORT))
        return client

    def GetMsg(self):
        recv_data = self.client.recv(1024)
        data = recv_data.decode(encoding='UTF-8',errors='strict')
        print("recv:", data)
        return data

    def SendMsg(self, xmldata):
        self.client.send(xmldata)

    def tcp_close(self):
        self.client.close()

    def __del__(self):
        self.client.close()
        self.client = None
        print("Robot instance destroyed")

class Xml_dispose:
    def __indent(self, elem, level=0):
        i = "\n" + level*"\t"
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "\t"
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                self.__indent(elem, level+1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    def ReadXml(self, file):
        tree = ET.parse(file)
        root = tree.getroot()
        xmldata = ET.tostring(root, encoding='utf-8')
        return xmldata

    def SetPos(self, file, element, attribute):
        tree = ET.parse(file)
        root = tree.getroot()
        sub = root.findall(element)[0]
        for key, val in attribute.items():
            sub.set(key, val)
        self.__indent(root)
        cur_dir = os.path.dirname(os.path.abspath(file))
        save_path = os.path.join(cur_dir, 'modify.xml')
        tree.write(save_path)
        return save_path
    
    def findattrib_fromstring(self, str, elem):
        root = ET.fromstring(str)        
        child = root.findall(elem)
        print(child[0].attrib)
        return child[0].attrib
    
    def findtext_fromstring(self, str, elem):
        root = ET.fromstring(str)        
        child = root.findall(elem)
        print(child[0].text)
        return child[0].text

class Yaml_dispose:
    def LoadYaml(self, file):
        with open(file, 'r', encoding='utf-8') as f:
            result = yaml.load(f, Loader=yaml.FullLoader)
        return result

    def WriteYaml(self, file, data):
        with open(file, 'w', encoding='utf-8') as f:
            yaml.dump(data=data, stream=f, allow_unicode=True, sort_keys=False, default_flow_style=True, width=149)

class Camera_connect:
    def __init__(self):
        logger.info("camera connecting......")
        try:
            self.cam = self.zmq_connect()
            logger.info("Camera connect successfully")
        except Exception as e:
            logger.info("Camera connect error:",e)

    def zmq_connect(self):
        context = zmq.Context()
        cam = context.socket(zmq.REQ)
        cam.connect('tcp://{0}:{1}'.format(zmq_IP, zmq_PORT))
        return cam
    
    def send_flag(self, flag):
        self.cam.send_string(flag)
        msg = self.cam.recv()
        print(msg)
        return msg
    
    def __del__(self):
        self.cam.close()
        self.cam = None
        print("Camera instance destroyed")