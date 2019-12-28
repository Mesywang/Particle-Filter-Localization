#include "load_map.h"



LoadMap::LoadMap(const char* map_str)
{
    this->map = (map_type*) malloc(sizeof(map_type));    //为地图存储分配内存
    ReadFromData(map_str);
}

LoadMap::~LoadMap()
{

}

int LoadMap::ReadFromData(const char* map_name)
{
    int x, y, count;
    float temp;
    char line[256];
    FILE *fp;

    if((fp = fopen(map_name, "rt")) == NULL)     //打开文件
    {
        fprintf(stderr, "ERROR: Could not open file %s\n", map_name);
        return -1;
    }
    fprintf(stderr, "# Reading map: %s\n", map_name);

    //fgets从指定的流 stream 读取一行存入line[],当读取255个字符时,或读取到换行符时,或到达文件末尾时,它会停止
    while((fgets(line, 256, fp) != NULL)
          && (strncmp("global_map[0]", line , 13) != 0))
    {
        if(strncmp(line, "robot_specifications->resolution", 32) == 0)
            //从字符串line[]中读取%d格式输入,并存入map->resolution中
            if(sscanf(&line[32], "%d", &(map->resolution)) != 0)
                printf("# Map resolution: %d cm\n", map->resolution);
        if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
            if(sscanf(&line[35], "%g", &(map->offset_x)) != 0) 
            {
                map->offset_x = map->offset_x;
                printf("# Map offsetX: %g cm\n", map->offset_x);
            }
        if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) 
        {
            if (sscanf(&line[35], "%g", &(map->offset_y)) != 0) 
            {
                map->offset_y = map->offset_y;
                printf("# Map offsetY: %g cm\n", map->offset_y);
            }
        }
    }

    if(sscanf(line,"global_map[0]: %d %d", &map->size_y, &map->size_x) != 2)  //如果成功则返回成功匹配和赋值的个数
    {
        fprintf(stderr, "ERROR: corrupted file %s\n", map_name);
        fclose(fp);
        return -1;
    }
    printf("# Map size: %d %d\n", map->size_x, map->size_y);


    map->prob = (float **)calloc(map->size_x, sizeof(float *));   //为二维数组分配内存，calloc同时将数组初始化为0
    for(int i = 0; i < map->size_x; i++)
    {
        map->prob[i] = (float *)calloc(map->size_y, sizeof(float));
    }
    
    map->min_x = map->size_x;
    map->max_x = 0;
    map->min_y = map->size_y;
    map->max_y = 0;
    count = 0;
    for(x = 0; x < map->size_x; x++)
    {
        for(y = 0; y < map->size_y; y++, count++)
        {
            if(count % 10000 == 0)
                fprintf(stderr, "\r# Reading ... (%.2f%%)", count / (float)(map->size_x * map->size_y) * 100);
            fscanf(fp,"%e", &temp);         //每次读取地图一个像素点的概率值

            if(temp < 0.0)
                map->prob[x][y] = -1;       //地图状态未知
            else
            {
                if(x < map->min_x)          //min~max指的是地图有效区域,默认min,max赋的都是最大值,其值在这里进行更新
                    map->min_x = x;
                else if(x > map->max_x)
                    map->max_x = x;
                if(y < map->min_y)
                    map->min_y = y;
                else if(y > map->max_y)
                    map->max_y = y;
                map->prob[x][y] = temp;    //地图概率赋值
            }
        }
    }
    //显示地图加载进度
    fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",count / (float)(map->size_x * map->size_y) * 100);
    fclose(fp);
    return 0;
}


void LoadMap::PublishMap(ros::NodeHandle node_)
{
    nav_msgs::OccupancyGrid ros_map; 
    ros_map.header.stamp = ros::Time::now();
    ros_map.header.frame_id = "map";
    ros_map.info.map_load_time = ros::Time::now();
    ros_map.info.resolution = this->map->resolution * 0.01;   //rviz中单位为m，1Ocm -> 0.1m
    ros_map.info.width = this->map->size_x;
    ros_map.info.height = this->map->size_y;
    ros_map.info.origin.position.x = 0.0;
    ros_map.info.origin.position.y = 0.0;
    ros_map.info.origin.position.z = 0.0;
    ros_map.info.origin.orientation.x = 0.0;
    ros_map.info.origin.orientation.y = 0.0;
    ros_map.info.origin.orientation.z = 0.0;
    ros_map.info.origin.orientation.w = 1.0;

    ros_map.data.resize(ros_map.info.width * ros_map.info.height);

    for(int x=0; x < ros_map.info.width; x++)
    {
        for(int y=0; y < ros_map.info.height; y++)
        {
            // ros_map.data[x*ros_map.info.width + y] = (unsigned int)(map->prob[y][x] * 255);
            ros_map.data[y*ros_map.info.height + x] = (uint8_t)(this->map->prob[x][y] * 255);
        }
    }

    map_pub = node_.advertise<nav_msgs::OccupancyGrid>("map", 1 ,true);
    map_meta_pub = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1 ,true);

    map_meta_pub.publish(ros_map.info);
    map_pub.publish(ros_map);
}


map_type* LoadMap::GetMap()
{
    return this->map;
}


