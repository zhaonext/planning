import xml.etree.ElementTree as ET
import os


def is_road_vertical(first_lane_points):
    # 判断道路是垂直还是水平
    return first_lane_points[0]['x'] == first_lane_points[-1]['x']


def generate_control_points(start, end):
    mid_x = (start['x'] + end['x']) / 2
    mid_y = (start['y'] + end['y']) / 2
    return [{'x': start['x'], 'y': start['y']}, {'x': mid_x, 'y': mid_y}, {'x': end['x'], 'y': end['y']}]


def get_lane_end_point(root, road_id, lane_id, is_start):
    path = f".//Road[@id='{road_id}']/Lane[@id='{lane_id}']/BezierCurve/ControlPoint"
    control_points = root.findall(path)
    if not control_points:
        return None
    index = 0 if is_start else -1
    return {'x': float(control_points[index].get('x')), 'y': float(control_points[index].get('y'))}


def calculate_additional_lanes(first_lane_points, lane_width, total_lanes=4):
    additional_lanes = []
    vertical = is_road_vertical(first_lane_points)

    # 生成正向车道（右侧），跳过第一条车道
    for lane in range(2, total_lanes + 1):
        new_lane = []
        for point in first_lane_points:
            offset = (lane - 1) * lane_width
            if vertical:
                new_point = {'x': point['x'] + offset, 'y': point['y']}
            else:
                new_point = {'x': point['x'], 'y': point['y'] - offset}
            new_lane.append(new_point)

        print(f"正向车道 {lane} 的控制点: {new_lane}")
        additional_lanes.append((str(lane), new_lane))

    # 生成反向车道（左侧）
    for lane in range(1, total_lanes + 1):
        new_lane = []
        for point in reversed(first_lane_points):  # 反向遍历控制点
            offset = lane * lane_width
            if vertical:
                new_point = {'x': point['x'] - offset, 'y': point['y']}
            else:
                new_point = {'x': point['x'], 'y': point['y'] + offset}
            new_lane.append(new_point)

        additional_lanes.append((f"-{lane}", new_lane))

    return additional_lanes


def update_zebra_crossings(root):
    front, back = 0, -1
    x_offset, y_offset = 1.875, 1.875
    for road_node in root.findall('.//Road'):
        road_id = int(road_node.get('id'))
        lanes = road_node.findall('.//Lane')

        if road_id == 1:
            # 查找对应的斑马线元素
            zebra_crossing = root.find(f".//ZebraCrossing[@id='{road_id}']")
            if zebra_crossing is not None:
                for lane in lanes:
                    lane_id = int(lane.get('id'))
                    control_points = lane.findall('.//ControlPoint')
                    if lane_id == 4 and control_points:
                        x_start = float(control_points[back].get('x')) - x_offset
                        y_start = float(control_points[back].get('y')) - y_offset
                        zebra_crossing.set('x_start', str(x_start))
                        zebra_crossing.set('y_start', str(y_start))
                    elif lane_id == -4 and control_points:
                        x_end = float(control_points[front].get('x')) - x_offset
                        y_end = float(control_points[back].get('y')) + y_offset
                        zebra_crossing.set('x_end', str(x_end))
                        zebra_crossing.set('y_end', str(y_end))
        if road_id == 2:
            # 查找对应的斑马线元素
            zebra_crossing = root.find(f".//ZebraCrossing[@id='{road_id}']")
            if zebra_crossing is not None:
                front, back = 0, -1
                for lane in lanes:
                    lane_id = int(lane.get('id'))
                    control_points = lane.findall('.//ControlPoint')

                    if lane_id == 4 and control_points:
                        x_start = float(control_points[back].get('x')) + x_offset
                        y_start = float(control_points[back].get('y')) - y_offset
                        zebra_crossing.set('x_start', str(x_start))
                        zebra_crossing.set('y_start', str(y_start))
                    elif lane_id == -4 and control_points:
                        x_end = float(control_points[front].get('x')) - x_offset
                        y_end = float(control_points[front].get('y')) - y_offset
                        zebra_crossing.set('x_end', str(x_end))
                        zebra_crossing.set('y_end', str(y_end))
        if road_id == 3:
            # 查找对应的斑马线元素
            zebra_crossing = root.find(f".//ZebraCrossing[@id='{road_id}']")
            if zebra_crossing is not None:
                for lane in lanes:
                    lane_id = int(lane.get('id'))
                    control_points = lane.findall('.//ControlPoint')

                    if lane_id == 4 and control_points:
                        x_start = float(control_points[front].get('x')) + x_offset
                        y_start = float(control_points[front].get('y')) - y_offset
                        zebra_crossing.set('x_start', str(x_start))
                        zebra_crossing.set('y_start', str(y_start))
                    elif lane_id == -4 and control_points:
                        x_end = float(control_points[back].get('x')) + x_offset
                        y_end = float(control_points[front].get('y')) + y_offset
                        zebra_crossing.set('x_end', str(x_end))
                        zebra_crossing.set('y_end', str(y_end))

        if road_id == 4:
            # 查找对应的斑马线元素
            zebra_crossing = root.find(f".//ZebraCrossing[@id='{road_id}']")
            if zebra_crossing is not None:
                for lane in lanes:
                    lane_id = int(lane.get('id'))
                    control_points = lane.findall('.//ControlPoint')

                    if lane_id == 4 and control_points:
                        x_start = float(control_points[back].get('x')) + x_offset
                        y_start = float(control_points[front].get('y')) + y_offset
                        zebra_crossing.set('x_start', str(x_start))
                        zebra_crossing.set('y_start', str(y_start))
                    elif lane_id == -4 and control_points:
                        x_end = float(control_points[front].get('x')) - x_offset
                        y_end = float(control_points[back].get('y')) + y_offset
                        zebra_crossing.set('x_end', str(x_end))
                        zebra_crossing.set('y_end', str(y_end))


def update_road_network(xml_file, updated_xml_file, lane_width):
    print(f"开始读取文件: {xml_file}")
    tree = ET.parse(xml_file)
    root = tree.getroot()

    roads = []
    for road in root.findall('.//Road'):
        road_id = road.get('id')
        print(f"处理道路: {road_id}")

        first_lane = road.find('.//Lane[@id="1"]')
        first_lane_points = [
            {'x': float(cp.get('x')), 'y': float(cp.get('y'))}
            for cp in first_lane.find('.//BezierCurve').findall('ControlPoint')
        ]
        print(f"正向车道 {road.find('.//Lane').get('id')} 的控制点: {first_lane_points}")

        roads.append({
            'id': road_id,
            'ControlPoint': first_lane_points
        })
        additional_lanes = calculate_additional_lanes(first_lane_points, lane_width)

        # 移除原有车道，添加新的车道
        for lane in road.findall('.//Lane')[1:]:
            road.remove(lane)
        for lane_id, lane_points in additional_lanes:
            lane_elem = ET.SubElement(road, 'Lane', id=lane_id)
            bezier_elem = ET.SubElement(lane_elem, 'BezierCurve')
            for point in lane_points:
                ET.SubElement(bezier_elem, 'ControlPoint', x=str(point['x']), y=str(point['y']))

    # 更新斑马线位置
    update_zebra_crossings(root)

    # 打印斑马线信息（可选）
    for crossing in root.find('.//ZebraCrossings').findall('ZebraCrossing'):
        print(
            f"斑马线 {crossing.get('id')}: 起点 ({crossing.get('x_start')}, {crossing.get('y_start')}), 终点 ({crossing.get('x_end')}, {crossing.get('y_end')})")

    # 更新道路连接
    for connection in root.findall('.//Connection'):
        from_road = connection.find('From').get('road')
        from_lane = connection.find('From').get('lane')
        to_road = connection.find('To').get('road')
        to_lane = connection.find('To').get('lane')

        start_point = get_lane_end_point(root, from_road, from_lane, is_start=False)
        end_point = get_lane_end_point(root, to_road, to_lane, is_start=True)

        control_points = generate_control_points(start_point, end_point)

        # control_points_element = connection.find('ControlPoints')
        # control_points_element.clear()

        # for point in control_points:
        #     ET.SubElement(control_points_element, 'ControlPoint', x=str(point['x']), y=str(point['y']))

        # 打印连接线及其控制点
        print(
            f"连接线{connection.get('id')}: From Road {from_road}: lane {from_lane} to Road {to_road}: lane {to_lane}")
        print(f"控制点: {control_points}")

    tree.write(updated_xml_file)
    print(f"文件更新完成并保存到: {updated_xml_file}")


# 获取脚本所在的目录
script_dir = os.path.dirname(__file__)

# 构建相对于脚本目录的文件路径
input_xml_file = os.path.join(script_dir, 'RoadNet.xml')
output_xml_file = os.path.join(script_dir, 'RoadNet.xml')

# 使用示例
print("更新道路网络控制点...")
update_road_network(input_xml_file, output_xml_file, 3.75)
print("更新完成！")
