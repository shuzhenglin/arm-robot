from kuka_controller import *

if __name__ == "__main__":
    flagxml = os.path.join(base_path, 'XML/Getpose.xml')
    axisyml = os.path.join(base_path, 'YAML/axis.yml')
    rob = Robot_connect()
    xml = Xml_dispose()
    yml = Yaml_dispose()

    xmldata = xml.ReadXml(flagxml)
    rob.SendMsg(xmldata)
    #* tell client set success
    msg = rob.GetMsg()
    rob.tcp_close()

    data = xml.findattrib_fromstring(msg, ".//Data//LastAXIS")
    
    yml.WriteYaml(axisyml, data)

    del rob