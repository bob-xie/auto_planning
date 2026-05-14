#include <iostream>
#include "app/AppManager.h"

int main(int argc, char** argv) {
    std::cout << "=== 室外乘用车自动驾驶系统启动 ===" << std::endl;
    
    AD::App::AppManager app;
    
    if (!app.init("Sedan")) {
        std::cerr << "应用初始化失败" << std::endl;
        return -1;
    }
    
    std::cout << "\n系统已准备就绪" << std::endl;
    std::cout << "请从菜单选择操作" << std::endl;
    
    app.run();
    
    return 0;
}