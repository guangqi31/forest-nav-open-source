"""在新林地启动真实G1/导航，通过Tk控件提交1米目标并保存面板验证结果。"""
import json,math,os,signal,subprocess,sys,time,tkinter as tk
from pathlib import Path
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent
os.environ.update(ROS_DOMAIN_ID='188',FOREST_NAV_STATE_PORT='15481',FOREST_NAV_COMMAND_PORT='15482',
    FOREST_TERRAIN_SCENE=str(PROJECT/'forest_terrain_3d/generated/basic/environment.usda'),
    FOREST_NAV_MAP=str(PROJECT/'terrain_framework_baseline/results/basic/terrain_costmap.npz'),
    FOREST_NAV_SENSOR_MESH=str(ROOT/'results/basic/sensor_obstacles.npz'),FOREST_NAV_PARAMS=str(ROOT/'navigation.yaml'))
sys.path.insert(0,str(PROJECT/'nav2_path_tracking'))
import rclpy
from forest_nav_control.coordinate_controller import CoordinateController,ControllerPanel
from forest_nav_control.start_navigation import simulation_environment
from preflight import prepare

def main():
    prepare('basic');children=[];logs=[];node=window=panel=None
    report={'passed':False,'backend':'Isaac Sim G1 + Nav2','tests':{}}
    def start(name,args,env=None):
        log=(ROOT/'results'/f'panel_{name}.log').open('w');logs.append(log)
        child=subprocess.Popen(args,cwd=PROJECT/'nav2_path_tracking',env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(child)
    def pump(predicate,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if any(child.poll() is not None for child in children):raise RuntimeError('面板测试子进程提前退出')
            window.update()
            if predicate():return
            time.sleep(.01)
        raise TimeoutError(f'面板测试超时：{node.event}')
    rclpy.init()
    try:
        start('navigation',['bash','start_tracking.sh'])
        start('g1',['bash','start_g1_policy.sh','--visualizer','kit'],simulation_environment())
        node=CoordinateController();window=tk.Tk();panel=ControllerPanel(window,node);window.geometry('+40+80');window.update()
        assert [v.get() for v in panel.values]==['1','0','0'],'新林地默认目标错误'
        assert str(panel.go_button['state'])=='disabled','未就绪时按钮应禁用'
        pump(lambda:node.ready and str(panel.go_button['state'])=='normal',240)
        start_pos=node.pose[:2]
        assert math.hypot(*start_pos)<.3,'出生点坐标错误'
        panel.go_button.invoke()
        pump(lambda:node.last_result is not None,120)
        pump(lambda:panel.progress.get().startswith('导航状态：已到达目标'),3)
        error=math.dist(node.pose[:2],(1,0));distance=math.dist(node.pose[:2],start_pos)
        # 与完整导航验收一致：Nav2 达标后再留一个控制周期的反馈量化余量。
        assert node.last_result==4 and error<.32 and distance>.5,(node.last_result,error,distance)
        report.update(passed=True,initial_pose=start_pos,final_pose=node.pose,goal=[1,0,0],position_error_m=error,displacement_m=distance,clicked_actual_tk_button=True)
        window.lift()
        # 等X11合成器提交最终Tk帧，避免截图抓到更新前的窗口内容。
        deadline=time.monotonic()+.4
        while time.monotonic()<deadline:
            window.update();time.sleep(.01)
        report['displayed_progress']=panel.progress.get()
        report['displayed_message']=panel.message.get()
        from PIL import ImageGrab
        box=(window.winfo_rootx(),window.winfo_rooty(),window.winfo_rootx()+window.winfo_width(),window.winfo_rooty()+window.winfo_height())
        ImageGrab.grab(bbox=box).save(ROOT/'results/panel_forest.png')
    except Exception as e:
        report['error']=f'{type(e).__name__}: {e}';raise
    finally:
        (ROOT/'results/panel_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if panel:panel.close()
        if node:node.destroy_node()
        rclpy.try_shutdown()
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        for child in children:
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
    print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
