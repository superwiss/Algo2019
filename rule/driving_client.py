from drive_controller import DrivingController


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False
        self.collision_flag = True
        self.sensing_info = None

        # 핸들로 꺽을 수 있는 최대 각도 (abs)
        self.MAX_STEERING_ANGLE = 50

        # 최대 제한 속도 (Km/h)
        self.MAX_SPEED = 40

        # 장애물을 회피하는 감지 거리 (미터)
        # 예를 들어 값이 25라면, 25m 이내의 장애물에 대해서만 회피한다.
        self.OBSTACLE_SENSING_DIST = 25

        # 장애물 좌우로부터 떨어져야 하는 거리
        # 예를 들어 값이 5라면, 장애물의 중심으로부터 좌/우 5m만큼 떨어져서 주행한다.
        self.OBSTACLE_EVASION_DIST = 5

        # 차량이 중앙선에서 떨어진 거리만큼 핸들을 돌려야 하는데, 얼만큼 돌릴지를 결정하는 인자.
        # 예를 들어 값이 10이라면, 중앙선에서 10m 떨어져 있으면 핸들을 최대로 꺾고, 0 ~ 10m 사이는 부드럽게 꺾는다.
        self.STEERING_BY_MIDDLE_FACTOR = 10

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()


    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #

        if self.is_debug:
            print("=========================================================")
            print("to middle: {}".format(sensing_info.to_middle))

            print("collided: {}".format(sensing_info.collided))
            print("car speed: {} km/h".format(sensing_info.speed))

            print("is moving forward: {}".format(sensing_info.moving_forward))
            print("moving angle: {}".format(sensing_info.moving_angle))
            print("lap_progress: {}".format(sensing_info.lap_progress))

            print("track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################

        self.sensing_info = sensing_info;

        # Moving straight forward

        # 핸들을 제어한다.
        car_controls.steering = self.get_steering()

        # 브레이크  / 엑셀을 제어한다.
        car_controls.brake = self.get_brake()
        if car_controls.brake > 0:
            car_controls.throttle = 0
        else:
            car_controls.throttle = 1

        if self.is_debug:
            print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        return car_controls

    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = "My_Engine"
        return player_name

    # ============================
    # 차량의 진행 방향을 계산한다.
    # ============================
    def get_steering(self):
        # 차량이 유지하려고 하는 가상의 중앙선을 계산한다. 우선 차량과 중앙선과의 거리를 기반으로 최초 값이 설정된다.
        virtual_dist_from_middle = self.sensing_info.to_middle 

        # 앞에 장애물이 있다면, 장애물을 피할 수 있도록 가상의 중앙선 위치를 보정해 준다.
        virtual_dist_from_middle -= self.get_fixed_middle_distance_from_closest_obstacle()

        # 차량 진행 방향을 기반으로 중앙선에 붙기 위한 방향 각도를 계산한다.
        fixed_angle_by_moving_angle = self.sensing_info.moving_angle * -1

        # 가상의 중앙선에서 멀리 떨어져 있으면 가까이 붙기 위해서 방향 각도를 보정한다.
        fixed_angle_by_middle_distance = virtual_dist_from_middle * -1 / self.STEERING_BY_MIDDLE_FACTOR * self.MAX_STEERING_ANGLE

        # 핸들 꺾는 각도 = 차량이 중앙선에 붙기 위해서 핸들을 꺾는 각도 + 차량의 진행 방향과 도로의 진행 방향을 수평으로 맞추기 위해서 핸들을 꺾는 각도
        result = fixed_angle_by_moving_angle + fixed_angle_by_middle_distance

        if self.is_debug:
            print("차량 각도 보정값: {}, 중앙차선 보정값: {}, Steering: {}".format(fixed_angle_by_moving_angle, fixed_angle_by_middle_distance, self.get_steering_by_angle(result)))

        return self.get_steering_by_angle(result)

    # ============================
    # 브레이크를 밟을지 말지 결정한다.
    # TODO all or nothing 방식이 아니라, 부드럽게 밟자.
    # ============================
    def get_brake(self):
        result = 0
        if self.sensing_info.speed > self.MAX_SPEED:
            result = 1

        # print("브레이크: {}".format(result))
        return result

    # ============================
    # 가장 가까운 장애물을 대상으로, 차량이 장애물의 좌우 몇 미터 지점을 지나가는게 좋을지 계산한다.
    # ============================
    def get_fixed_middle_distance_from_closest_obstacle(self):
        result = 0

        if len(self.sensing_info.track_forward_obstacles) != 0:
            # 가장 가까운 장애물 하나를 가져온다.
            closest = self.sensing_info.track_forward_obstacles[0]

            # 너무 멀리 떨어진 장애물에 대해서는 반응하지 않고,
            # 장애물 감지 거리(self.OBSTACLE_SENSING_DIST) 이내의 장애물에 대해서만 회피한다.
            if closest['dist'] < self.OBSTACLE_SENSING_DIST:
                if closest['to_middle'] >= 0:
                    result = closest['to_middle'] - self.OBSTACLE_EVASION_DIST
                else:
                    result = closest['to_middle'] + self.OBSTACLE_EVASION_DIST

                if self.is_debug:
                    print("장애물 발견: {}, 보정값: {}".format(closest, result))

        return result

    # ============================
    # 희망하는 진행 각도(angle)로부터 핸들 꺽는 방향과 힘(steering)을 계산한다.
    # ============================
    def get_steering_by_angle(self, angle):
        if angle < -1 * self.MAX_STEERING_ANGLE:
            # 희망하는 진행 각도가 -50도 보다 크다면, 핸들을 최대한 왼쪽으로 꺾는다.
            return -1
        elif angle > self.MAX_STEERING_ANGLE:
            # 희망하는 진행 각도가 50도 보다 크다면, 핸들을 최대한 오른쪽으로 꺾는다.
            return 1
        else:
            # 희망하는 진행 각도가 -50도 ~ 50도 사이라면, 핸들을 부드럽게 꺾는다.
            return angle / self.MAX_STEERING_ANGLE

if __name__ == '__main__':
    client = DrivingClient()
    client.run()
