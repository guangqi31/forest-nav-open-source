"""汇总真实速度试验，区分稳定性、速度跟踪、启动故障和首次失败条件。"""
import argparse
import csv
import json
import math
from pathlib import Path


def build(folder):
    protocol=json.loads((folder/'protocol.json').read_text())
    summary=json.loads((folder/'summary.json').read_text()) if (folder/'summary.json').exists() else {}
    trials=[]
    for f in sorted(folder.glob('v*/result.json')):
        result=json.loads(f.read_text());samples=f.parent/'samples.jsonl'
        rows=[json.loads(s) for s in samples.read_text().splitlines() if s.strip()]
        brake=[r for r in rows if r['phase'] in ('brake','stop')]
        if brake:
            first=brake[0]['state']
            settled=next((r['state'] for r in brake if r['state'].get('locomotion',{}).get('settled')),None)
            # 从开始降低指令计算累计路程，包含减速段；不将纯零指令段位移冒充总刹车距离。
            end=settled['sim_time'] if settled else brake[-1]['state']['sim_time']
            samples_before=[r['state'] for r in brake if r['state']['sim_time']<=end]
            result['braking_path_length_m']=sum(math.dist(a['pose'][:2],b['pose'][:2]) for a,b in zip(samples_before,samples_before[1:]))
            result['settled_after_brake_start_sim_sec']=end-first['sim_time'] if settled else None
        result['trial']=f.parent.name
        trials.append(result)
    by_speed={}
    for trial in trials:by_speed.setdefault(trial['command_mps'],[]).append(trial)
    completed=[(speed,items) for speed,items in by_speed.items()
               if len(items)==protocol['repeats'] and all(x.get('stability_passed') for x in items)]
    best=max(completed,key=lambda x:min(t['stages']['straight']['mean_forward_mps'] for t in x[1])) if completed else None
    assessment=dict(source=str(folder),all_trials_completed=summary.get('completed',False),
        confirmed_command_mps=best[0] if best else None,
        confirmed_actual_min_mps=min(t['stages']['straight']['mean_forward_mps'] for t in best[1]) if best else None,
        confirmed_actual_max_mps=max(t['stages']['straight']['mean_forward_mps'] for t in best[1]) if best else None,
        first_failed_command_mps=summary.get('first_failed_command_mps'),
        trials=trials,claim='有限时间、平地和规定转弯/停车条件下的已测通过档位；不等于绝对不倒极限。')
    (folder/'assessment.json').write_text(json.dumps(assessment,ensure_ascii=False,indent=2))
    lines=['# G1平地速度标定结果','',f'结果目录：{folder}','',
           '使用现有ONNX，未训练，未修改正式导航限速。各速度两轮，种子42/43。',
           '指令速度与实际速度分开记录；通过结果只适用于本轮平地、接触配置及有限采样窗口。','',
           '|指令速度 m/s|轮次|实际直行均速 m/s（末2秒）|稳定性|速度跟踪|自减速开始至站稳路程 m|问题|',
           '|---|---|---|---|---|---|---|']
    for t in trials:
        measured=t.get('stages',{}).get('straight',{}).get('mean_forward_mps')
        distance=t.get('braking_path_length_m')
        lines.append(f"|{t['command_mps']:.2f}|{t['seed']}|{measured:.3f}" if measured is not None else f"|{t['command_mps']:.2f}|{t['seed']}|无有效样本")
        lines[-1]+=f"|{'通过' if t.get('stability_passed') else '未通过'}|{'通过' if t.get('speed_tracking_passed') else '未通过/未完成'}|{format(distance,'.3f') if distance is not None else '未完成'}|{', '.join(t['issues']) or '无'}|"
    lines+=['','## 判读','']
    if best:
        lines.append(f"两轮稳定通过的最高实测档位：指令{best[0]:.2f}米/秒，实际{assessment['confirmed_actual_min_mps']:.3f}～{assessment['confirmed_actual_max_mps']:.3f}米/秒。")
    if not summary.get('completed'):lines.append('测试尚在进行，不能据此给出最大速度。')
    elif summary.get('first_failed_command_mps') is None:lines.append('达到本轮搜索上限，未找到失败边界；不能把搜索上限当成机器人极限。')
    else:
        failed=[t for t in trials if not t.get('stability_passed')]
        lines.append(f"首次未通过指令档位：{summary['first_failed_command_mps']:.2f}米/秒。具体原因：{', '.join(failed[-1]['issues']) if failed else '见日志'}。")
        if not summary.get('physical_failure_preserved'):lines.append('此项不一定是摔倒，须按失败原因区分跟踪、方向、停车或启动故障。')
    lines+=['','本轮尚未二分细化、长距离耐久或林地测试；不写回正式导航参数。',
            '停止距离包含以0.8米/秒²降低速度指令的过程，不是瞬间归零或硬件急停距离。']
    (folder/'标定结果.md').write_text('\n'.join(lines)+'\n')
    return {k:v for k,v in assessment.items() if k!='trials'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path)
    print(json.dumps(build(parser.parse_args().folder),ensure_ascii=False))
