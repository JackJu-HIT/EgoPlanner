/*
 * @Function:Using Ego Planner to Generate Trajectory
 * @Create by:juchunyu@qq.com
 * @Date:2025-08-02 18:58:01
 */
#include <iostream> 
#include <string>    
#include <ios>     

#include "planner_interface.h"
#include "./planner/matplotlib-cpp/matplotlibcpp.h"

using namespace ego_planner;

namespace plt = matplotlibcpp;


int main()
{
    std::vector<double> global_x;
    std::vector<double> global_y;

    std::vector<double> local_plan_x;
    std::vector<double> local_plan_y;

    std::vector<double> obs_x;
    std::vector<double> obs_y;
    std::vector<double> color;

    std::shared_ptr<PlannerInterface>  plan = std::make_shared<PlannerInterface>();
    
    //初始化控制参数
    double max_vel = 1.0;
    double max_acc = 1.0;
    double max_jerk = 1.0;
    plan->initParam(max_vel,max_acc,max_jerk);

    //初始化ESDF地图
    double resolution = 0.1;
    double x_size = 10.0;
    double y_size = 10.0;
    double z_size = 10.0;
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    double inflateValue = 0.0;
    plan->initEsdfMap(x_size,y_size,z_size,resolution,origin,inflateValue);
  
    //添加障碍物点云
    std::vector<ObstacleInfo> obstacle;
    ObstacleInfo Obstemp;
    Obstemp.x = 6;
    Obstemp.y = 6;
    obstacle.push_back(Obstemp);
    Obstemp.x = 3.0;
    Obstemp.y = 3.0;
    obstacle.push_back(Obstemp);

    plan->setObstacles(obstacle);

    //添加全局路径点
    std::vector<PathPoint> global_plan_traj;
    
    global_x.clear();
    global_y.clear();

    float j = 0;

    while(j < 10)
    {
        PathPoint tempPoint;
        tempPoint.x = j;
        tempPoint.y = j;

        global_plan_traj.push_back(tempPoint);
  
        j+= 0.1;
        global_x.push_back(j);
        global_y.push_back(j);
    }


    plan->setPathPoint(global_plan_traj);

    
    //开始规划
    plan->makePlan();
    
    //获取规划结果
    std::vector<PathPoint> plan_traj_res;
    plan->getLocalPlanTrajResults(plan_traj_res);
    
    local_plan_x.clear();
    local_plan_y.clear();

    for(int i = 0; i < plan_traj_res.size(); i++)
    {
        local_plan_x.push_back(plan_traj_res[i].x);
        local_plan_y.push_back(plan_traj_res[i].y);
    }

    //show
    for(int i = 0; i < obstacle.size();i++)
    {
        obs_x.push_back(obstacle[i].x);
        obs_y.push_back(obstacle[i].y);
        color.push_back(1.0);
    }

    std::map<std::string, std::string> keywords1;
    keywords1.insert(std::pair<std::string, std::string>("label", "globaltraj"));
    keywords1.insert(std::pair<std::string, std::string>("linewidth", "3.5"));
    plt::plot(global_x, global_y, keywords1);

    std::map<std::string, std::string> keywords2;
    keywords2.insert(std::pair<std::string, std::string>("label", "localtraj"));
    keywords2.insert(std::pair<std::string, std::string>("linewidth", "3.5"));
    plt::plot(local_plan_x, local_plan_y, keywords2);
  
    
    double point_size = 100.0;
    plt::scatter_colored(obs_x, obs_y,color,point_size,{{"cmap", "viridis"}});
    plt::legend();
    plt::title("Ego Planner Results!");
    plt::show();


    return 0;
}