#!/usr/bin/env python3
"""Read-only YOLO + GraspNet viewer. No ROS, camera or motor access."""
import argparse
import json
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,default=Path.home()/'go2_diploma/sim2real/runtime/detection_debug')
    parser.add_argument('--grasp-file',type=Path,help='Default: sibling grasp_debug/latest.npz')
    parser.add_argument('--max-grasps',type=int,default=80)
    args=parser.parse_args()
    if args.max_grasps<1:parser.error('--max-grasps must be positive')
    pointer=args.directory/'latest.json'
    grasp_file=args.grasp_file or args.directory.parent/'grasp_debug/latest.npz'
    arm_file=grasp_file.parent/'latest_arm.json'
    root=tk.Tk();root.title('Go2 — YOLO + GraspNet (read only)');root.geometry('1500x800')
    panes=ttk.Panedwindow(root,orient=tk.HORIZONTAL);panes.pack(fill=tk.BOTH,expand=True)
    yolo=ttk.Frame(panes);grasp=ttk.Frame(panes)
    panes.add(yolo,weight=1);panes.add(grasp,weight=2)
    ttk.Label(yolo,text='YOLO — latest processed frame').pack()
    label=ttk.Label(yolo,text='Waiting for detection…',anchor='center');label.pack(fill=tk.BOTH,expand=True)
    status=ttk.Label(yolo,text=str(pointer),wraplength=420);status.pack(fill=tk.X)
    tabs=ttk.Notebook(grasp);tabs.pack(fill=tk.BOTH,expand=True)
    grasp_tab=ttk.Frame(tabs);arm_tab=ttk.Frame(tabs)
    tabs.add(grasp_tab,text='GraspNet / camera');tabs.add(arm_tab,text='Arm / approach')
    controls=ttk.Frame(grasp_tab);controls.pack(fill=tk.X)
    ttk.Label(controls,text='GraspNet stage:').pack(side=tk.LEFT)
    stage=tk.StringVar(value='decoded')
    selector=ttk.Combobox(controls,textvariable=stage,state='readonly',
        values=('decoded','cloud','bbox','width','depth','target_binding'),width=12)
    selector.pack(side=tk.LEFT,padx=8)
    ttk.Label(controls,text='Saved calculation; independent of latest YOLO frame').pack(side=tk.LEFT)
    grasp_status=ttk.Label(grasp_tab,text='Waiting for a GraspNet calculation…',wraplength=950)
    grasp_status.pack(fill=tk.X)
    try:
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        from grasp_viewer import load_snapshot,draw_snapshot,STAGES
        fig=Figure(figsize=(10,6),dpi=100)
        ax2=fig.add_subplot(121);ax3=fig.add_subplot(122,projection='3d')
        fig.subplots_adjust(left=.04,right=.95,wspace=.25)
        canvas=FigureCanvasTkAgg(fig,master=grasp_tab)
        canvas.get_tk_widget().pack(fill=tk.BOTH,expand=True)
        toolbar=NavigationToolbar2Tk(canvas,grasp_tab,pack_toolbar=False);toolbar.pack(fill=tk.X)
    except Exception as error:
        grasp_status.configure(text=f'GraspNet viewer unavailable: {error}')
        canvas=None;selector.configure(state='disabled')
    arm_controls=ttk.Frame(arm_tab);arm_controls.pack(fill=tk.X)
    ttk.Label(arm_controls,text='Candidate:').pack(side=tk.LEFT)
    arm_choice=tk.StringVar()
    arm_selector=ttk.Combobox(arm_controls,textvariable=arm_choice,state='readonly',width=24)
    arm_selector.pack(side=tk.LEFT)
    arm_status=ttk.Label(arm_tab,text='Waiting for a new debug planning result',wraplength=950)
    arm_status.pack(fill=tk.X)
    try:
        from arm_plan_viewer import load_arm_snapshot,draw_arm_snapshot
        arm_fig=Figure(figsize=(9,7),dpi=100);arm_ax=arm_fig.add_subplot(111,projection='3d')
        arm_ax.view_init(elev=23,azim=-65)
        arm_canvas=FigureCanvasTkAgg(arm_fig,master=arm_tab)
        arm_canvas.get_tk_widget().pack(fill=tk.BOTH,expand=True)
        arm_toolbar=NavigationToolbar2Tk(arm_canvas,arm_tab,pack_toolbar=False)
        arm_toolbar.pack(fill=tk.X)
    except Exception as error:
        arm_status.configure(text=f'Arm viewer unavailable: {error}');arm_canvas=None
    arm_stamp=None;arm_data=None;arm_rendered=None
    last=None;original=None;last_size=None;snapshot=None;grasp_stamp=None;rendered=None

    def refresh():
        nonlocal last,original,last_size,snapshot,grasp_stamp,rendered
        nonlocal arm_stamp,arm_data,arm_rendered
        try:
            meta=json.loads(pointer.read_text());path=meta['image_path']
            version=(path,meta['saved_wall_ns'])
            changed=version!=last
            if changed:
                with Image.open(path) as image:original=image.copy()
                last=version
            size=(max(100,yolo.winfo_width()-12),max(100,label.winfo_height()-12))
            if original is not None and (changed or size!=last_size):
                image=original.copy();image.thumbnail(size)
                photo=ImageTk.PhotoImage(image,master=root)
                label.configure(image=photo,text='');label.image=photo;last_size=size
            age=(time.time_ns()-meta['saved_wall_ns'])/1e9
            status.configure(text=f'YOLO request {meta.get("operation_id", "?")} | {age:.1f}s ago | dropped={meta.get("dropped",0)}')
        except FileNotFoundError:status.configure(text='Waiting for detection. Camera starts after search pose.')
        except Exception as error:status.configure(text=f'YOLO: {error}')
        if canvas is not None and tabs.select()==str(grasp_tab):
            try:
                current=grasp_file.stat().st_mtime_ns
                if current!=grasp_stamp:
                    snapshot=load_snapshot(grasp_file);grasp_stamp=current
                key=(grasp_stamp,stage.get())
                if snapshot is not None and key!=rendered:
                    draw_snapshot(ax2,ax3,snapshot,stage.get(),args.max_grasps)
                    counts=' → '.join(f'{name}: {len(snapshot[name])}' for name in STAGES)
                    grasp_status.configure(text=f'GraspNet request {snapshot["metadata"]["request_id"]} | {counts}')
                    canvas.draw_idle();rendered=key
            except FileNotFoundError:grasp_status.configure(text='Waiting for a GraspNet calculation in debug mode…')
            except Exception as error:grasp_status.configure(text=f'GraspNet: {error}')
        if arm_canvas is not None and tabs.select()==str(arm_tab):
            try:
                current=arm_file.stat().st_mtime_ns
                if current!=arm_stamp:
                    arm_data=load_arm_snapshot(arm_file);arm_stamp=current
                    names=[c['candidate'] for c in arm_data['candidates']]
                    arm_selector.configure(values=names)
                    if arm_choice.get() not in names:
                        arm_choice.set(arm_data.get('selected') or (names[0] if names else ''))
                key=(arm_stamp,arm_choice.get())
                if arm_data is not None and key!=arm_rendered:
                    draw_arm_snapshot(arm_ax,arm_data,arm_choice.get())
                    arm_canvas.draw_idle();arm_rendered=key
                age=(time.time_ns()-arm_data['saved_wall_ns'])/1e9
                arm_status.configure(text=f'Request {arm_data["request_id"]} | {arm_data["status"]} | '
                    f'{age:.1f}s ago | Saved planning state, NOT live robot motion. '
                    'Object shape unknown; failed IK is not an executable pose.')
            except FileNotFoundError:
                arm_status.configure(text='Waiting for a NEW grasp plan with grasp_debug_enabled=true. Old snapshots have no arm state.')
            except Exception as error:arm_status.configure(text=f'Arm snapshot: {error}')
        root.after(500,refresh)
    refresh();root.mainloop()


if __name__=='__main__':main()
