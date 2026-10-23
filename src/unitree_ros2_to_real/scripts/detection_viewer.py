#!/usr/bin/env python3
"""Read-only viewer of processed YOLO images. No ROS, camera or motor access."""
import argparse
import json
from pathlib import Path
import tkinter as tk
from PIL import Image, ImageTk


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',default=str(Path.home()/'go2_diploma/sim2real/runtime/detection_debug'))
    args=parser.parse_args();pointer=Path(args.directory)/'latest.json'
    root=tk.Tk();root.title('Go2 camera — processed YOLO frames (read only)')
    label=tk.Label(root,text='Waiting for a processed frame…');label.pack()
    status=tk.Label(root,text=str(pointer));status.pack()
    last=None
    def refresh():
        nonlocal last
        try:
            meta=json.loads(pointer.read_text());path=meta['image_path']
            if path!=last:
                with Image.open(path) as image:
                    image.thumbnail((1100,800));photo=ImageTk.PhotoImage(image.copy())
                label.configure(image=photo,text='');label.image=photo;last=path
            import time
            age=(time.time_ns()-meta['saved_wall_ns'])/1e9
            status.configure(text=f"Saved frame — {age:.1f}s ago | {Path(path).name} | dropped={meta.get('dropped',0)}")
        except FileNotFoundError:status.configure(text='Waiting for detection. Camera starts after search pose.')
        except Exception as error:status.configure(text=str(error))
        root.after(500,refresh)
    refresh();root.mainloop()


if __name__=='__main__':main()
