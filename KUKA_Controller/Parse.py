import xml.etree.ElementTree as ET

tree = ET.parse(r"D:\KUKA\KUKA_Controller\XML\BinaryFixed.xml")

root = tree.getroot()

# print('root_tag', root.tag)

def indent(elem, level=0):
    i = "\n" + level*"\t"
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "\t"
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

if __name__ == "__main__":
    xml_string = ET.tostring(root, encoding='utf-8')
    #print(xml_string)
    for child in root.findall(".//CONFIGURATION//EXTERNAL"):
        print(child.tag, child.attrib)
        print(child[0].text)
        print(child[1].text)

    #* 增加节点
    sub1 = ET.SubElement(child, "KUKA")
    sub1.text = "szl"

    #* 修改Element
    child[0].text = "192.168.1.10"

    #* 删除节点
    child.remove(sub1)

    #* 修改节点属性
    sub2 = root.findall(".//RECEIVE//RAW//ELEMENT")[0]
    sub2.set("Set_Flag", "3")

    #* 重写xml
    indent(root)
    tree.write('test.xml')
    
    # for element in root.findall(".//RECEIVE//RAW"):
    #     print(element.tag, element.attrib)
    #     attribute =  element[0].attrib
    # for key, val in attribute.items():
    #     print(key, val)

    # elem_list = []
    # for element in root.iter('ELEMENT'):
    #     elem_list.append(element.attrib)

    # for i,elem in enumerate(elem_list):
    #     print('element{}'.format(i))
    #     for key, val in elem.items():
    #         print(key, val)