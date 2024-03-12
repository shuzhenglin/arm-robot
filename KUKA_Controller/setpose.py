from kuka_controller import *


if __name__ == "__main__":
    basexml = os.path.join(base_path, 'XML/Setpose.xml')
    armposfile = os.path.join(base_path, 'YAML/target_pose.yml')
    rob = Robot_connect()
    xml = Xml_dispose()
    yml = Yaml_dispose()
    attribute = {}
    attribute = yml.LoadYaml(armposfile)
    print(attribute)
    #* 修改Setpose.xml内容
    mfxml = xml.SetPos(basexml, ".//Robot//Pos", attribute)
    xmldata = xml.ReadXml(mfxml)
    rob.SendMsg(xmldata)
    rob.tcp_close()

    del rob