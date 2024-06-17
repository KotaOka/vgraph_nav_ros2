import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer_ = self.create_timer(1.0, self.publish_map)

        # Load map.yaml file
        with open('/home/ubuntu/ros2_ws/install/vgraph_nav_ros2/share/vgraph_nav_ros2/maps/map.yaml', 'r') as file:
            map_data = yaml.safe_load(file)

        # Parse map.yaml data
        map_metadata = map_data['image']
        resolution = map_data['resolution']
        origin = map_data['origin']

        # Example: Assuming you have 'data' field in map.yaml representing the grid data
        grid_data = map_data['data']

        # Create OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.width = map_metadata['width']
        occupancy_grid.info.height = map_metadata['height']
        occupancy_grid.info.resolution = resolution
        occupancy_grid.info.origin.position.x = origin['position']['x']
        occupancy_grid.info.origin.position.y = origin['position']['y']
        occupancy_grid.info.origin.position.z = origin['position']['z']
        occupancy_grid.info.origin.orientation.x = origin['orientation']['x']
        occupancy_grid.info.origin.orientation.y = origin['orientation']['y']
        occupancy_grid.info.origin.orientation.z = origin['orientation']['z']
        occupancy_grid.info.origin.orientation.w = origin['orientation']['w']
        occupancy_grid.data = grid_data

        self.occupancy_grid = occupancy_grid

    def publish_map(self):
        self.publisher_.publish(self.occupancy_grid)
        self.get_logger().info('Publishing map data')

def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

