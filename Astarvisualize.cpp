#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <sstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <unistd.h>     // For usleep()
#include <fcntl.h>      // For file control (O_RDWR, O_NOCTTY, etc.)
#include <termios.h>    // For serial port settings

const double CELL_SIZE = 0.05; // 5cm per cell
const double LINEAR_VELOCITY = 0.03; // 0.07 m/s

struct Node {
    int x, y;
    double g, h, f;
    Node* parent;

    Node(int x, int y, Node* parent = nullptr) : x(x), y(y), g(0), h(0), f(0), parent(parent) {}

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

std::string getKey(int x, int y) {
    std::stringstream ss;
    ss << x << "," << y;
    return ss.str();
}

bool isValid(int x, int y, const std::vector<std::vector<double>>& costmap) {
    int rows = costmap.size();
    int cols = costmap[0].size();
    return x >= 0 && x < rows && y >= 0 && y < cols && costmap[x][y] < 1.0;
}

double heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

std::vector<Node> reconstructPath(Node* node) {
    std::vector<Node> path;
    while (node != nullptr) {
        path.push_back(*node);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node> aStar(const std::vector<std::vector<double>>& costmap, int startX, int startY, int goalX, int goalY) {
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    double move_cost[] = {1.4, 1.0, 1.4, 1.0, 1.0, 1.4, 1.0, 1.4};

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<std::string, bool> closed;

    Node* start = new Node(startX, startY);
    start->g = 0;
    start->h = heuristic(startX, startY, goalX, goalY);
    start->f = start->g + start->h;

    open.push(*start);

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        std::string key = getKey(current.x, current.y);
        if (closed[key]) continue;
        closed[key] = true;

        if (current.x == goalX && current.y == goalY) {
            return reconstructPath(&current);
        }

        for (int i = 0; i < 8; i++) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!isValid(nx, ny, costmap)) continue;

            std::string nkey = getKey(nx, ny);
            if (closed[nkey]) continue;

            Node* neighbor = new Node(nx, ny, new Node(current));
            neighbor->g = current.g + move_cost[i] * costmap[nx][ny];
            neighbor->h = heuristic(nx, ny, goalX, goalY);
            neighbor->f = neighbor->g + neighbor->h;

            open.push(*neighbor);
        }
    }

    return {};
}

void visualizePath(const std::vector<std::vector<double>>& costmap, const std::vector<Node>& path, 
                   int startX, int startY, int goalX, int goalY) {
    const int cell_size = 30; // Reduced from 50 to fit better on screen
    const int rows = costmap.size();
    const int cols = costmap[0].size();
    
    // Create a white image
    cv::Mat image(rows * cell_size, cols * cell_size, CV_8UC3, cv::Scalar(255, 255, 255));
    
    // Draw grid lines
    for (int i = 0; i <= rows; ++i) {
        cv::line(image, cv::Point(0, i * cell_size), cv::Point(cols * cell_size, i * cell_size), 
                cv::Scalar(200, 200, 200), 1);
    }
    for (int j = 0; j <= cols; ++j) {
        cv::line(image, cv::Point(j * cell_size, 0), cv::Point(j * cell_size, rows * cell_size), 
                cv::Scalar(200, 200, 200), 1);
    }
    
    // Draw obstacles (cells with cost >= 1.0)
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (costmap[i][j] >= 1.0) {
                cv::rectangle(image, 
                             cv::Point(j * cell_size, i * cell_size),
                             cv::Point((j + 1) * cell_size, (i + 1) * cell_size),
                             cv::Scalar(0, 0, 0), cv::FILLED);
            }
        }
    }
    
    // Draw path
    if (!path.empty()) {
        for (size_t k = 0; k < path.size() - 1; ++k) {
            int x1 = path[k].x;
            int y1 = path[k].y;
            int x2 = path[k + 1].x;
            int y2 = path[k + 1].y;
            
            cv::line(image,
                    cv::Point(y1 * cell_size + cell_size/2, x1 * cell_size + cell_size/2),
                    cv::Point(y2 * cell_size + cell_size/2, x2 * cell_size + cell_size/2),
                    cv::Scalar(0, 0, 255), 3);
        }
        
        // Draw path points
        for (const auto& node : path) {
            cv::circle(image, 
                      cv::Point(node.y * cell_size + cell_size/2, node.x * cell_size + cell_size/2),
                      3, cv::Scalar(0, 255, 0), cv::FILLED);
        }
    }
    
    // Draw start and goal
    cv::circle(image, 
              cv::Point(startY * cell_size + cell_size/2, startX * cell_size + cell_size/2),
              5, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::circle(image, 
              cv::Point(goalY * cell_size + cell_size/2, goalX * cell_size + cell_size/2),
              5, cv::Scalar(0, 0, 255), cv::FILLED);
    
    // Add text labels
    cv::putText(image, "Start", 
               cv::Point(startY * cell_size + cell_size/2 + 10, startX * cell_size + cell_size/2),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(image, "Goal", 
               cv::Point(goalY * cell_size + cell_size/2 + 10, goalX * cell_size + cell_size/2),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    // Show image
    cv::imshow("A* Path Planning - 20x19 Map", image);
    cv::waitKey(0);
}



void calculateVelocityCommands(const std::vector<Node>& path, double initial_orientation_deg) {
    
    if (path.size() < 5) {//"/dev/ttyACM0"cout << "Path is too short to reach the 5th point." << std::endl;
        return;
    }

    // Get current (start) position and 5th point
    Node current = path[0];
    Node fifth_point = path[4]; // 0-based index, so path[4] is the 5th point

    // Convert positions to meters (X = columns, Y = rows)
    double current_x = current.y * CELL_SIZE; // X = horizontal (columns)
    double current_y = current.x * CELL_SIZE; // Y = vertical (rows)
    double target_x = fifth_point.y * CELL_SIZE;
    double target_y = fifth_point.x * CELL_SIZE;

    // Calculate desired heading angle (in radians)
    double dx = target_x - current_x;
    double dy = target_y - current_y;
    double desired_heading = atan2(dx, -dy); // atan2(dx, -dy) because "up" is -Y

    // Convert initial orientation to radians (0° = downward)
    double initial_orientation_rad = (initial_orientation_deg-180)* M_PI / 180.0;

    // Calculate angle difference (shortest turn)
    double angle_diff = desired_heading - initial_orientation_rad;
    
    // Normalize angle to [-π, π] (ensures shortest turn)
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    // Compute distance to target
    double distance = sqrt(dx * dx + dy * dy);

    // Calculate turning radius (R = distance / (2 * sin(angle_diff)))
    // Avoid division by zero and handle small angles
    double R;
    if (abs(angle_diff) < 0.01) {
        R = 1e6; // Approximate straight line (very large radius)
    } else {
        R = distance / (2 * sin(angle_diff));
    }

    // Calculate angular velocity (ω = v / R)
    double omega = LINEAR_VELOCITY / R;

    // Limit omega to avoid extreme turns (e.g., max 1.0 rad/s)
    double MAX_OMEGA = 1.0;
    if (abs(omega) > MAX_OMEGA) {
        omega = (omega > 0) ? MAX_OMEGA : -MAX_OMEGA;
    }

    // Output results
    std::cout << "Current position: (" << current.x << ", " << current.y << ")" << std::endl;
    std::cout << "5th point position: (" << fifth_point.x << ", " << fifth_point.y << ")" << std::endl;
    std::cout << "Initial orientation: " << initial_orientation_deg << " degrees" << std::endl;
    std::cout << "Desired heading: " << desired_heading * 180.0 / M_PI << " degrees" << std::endl;
    std::cout << "Angle difference: " << angle_diff * 180.0 / M_PI << " degrees" << std::endl;
    std::cout << "Distance to target: " << distance << " m" << std::endl;
    std::cout << "Turning radius (R): " << R << " m" << std::endl;
    std::cout << "Required angular velocity (ω): " << omega << " rad/s" << std::endl;
    std::cout << "Linear velocity (v): " << LINEAR_VELOCITY << " m/s" << std::endl;
    

}

int main() {
    std::vector<std::vector<double>> costmap = {
        {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    };

    int startX = 0, startY = 0;
    int goalX = 6, goalY = 6;

    std::vector<Node> path = aStar(costmap, startX, startY, goalX, goalY);

    if (path.empty()) {
        std::cout << "No path found!" << std::endl;
    } else {
        // Assume initial orientation is 0 degrees (pointing downwards)
        double initial_orientation_deg = 0.0;
        calculateVelocityCommands(path, initial_orientation_deg);
    }

    return 0;
}
