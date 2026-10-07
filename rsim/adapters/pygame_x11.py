"""Plain XPutImage presentation for forwarded SDL windows, without MIT-SHM.

SDL 2.28's framebuffer probes XShmAttach even over SSH and does not recover
from BadRequest. Keep SDL's window/input but upload pygame's pixels explicitly.
"""

import os


class X11Surface:
    def __init__(self, pg, title, size):
        from pygame._sdl2.video import Window
        from Xlib import X
        from Xlib.display import Display

        self.pg, self.X = pg, X
        self.window = Window(title, size=size)
        self.display = Display()
        pid_atom = self.display.intern_atom("_NET_WM_PID")

        def find(parent, depth=0):
            for window in parent.query_tree().children:
                value = window.get_full_property(pid_atom, X.AnyPropertyType)
                if value is not None and list(value.value) == [os.getpid()]:
                    return window
                if depth < 4:
                    found = find(window, depth + 1)
                    if found is not None:
                        return found
            return None

        self.drawable = find(self.display.screen().root)
        if self.drawable is None:
            raise RuntimeError("cannot find SDL window on the forwarded X display")
        self.depth = self.drawable.get_geometry().depth
        formats = self.display.display.info.pixmap_formats
        if self.depth not in (24, 32) or not any(
            f.depth == self.depth and f.bits_per_pixel == 32 for f in formats
        ):
            raise RuntimeError("forwarded pygame window requires a 24/32-bit X display")
        visual_id = self.drawable.get_attributes().visual
        visual = next(
            v
            for d in self.display.screen().allowed_depths
            for v in d.visuals
            if v.visual_id == visual_id
        )
        if (visual.red_mask, visual.green_mask, visual.blue_mask) != (
            0xFF0000,
            0xFF00,
            0xFF,
        ):
            raise RuntimeError("forwarded pygame window requires an RGB X visual")
        self.encoding = (
            "BGRA"
            if self.display.display.info.image_byte_order == X.LSBFirst
            else "ARGB"
        )
        self.gc = self.drawable.create_gc(graphics_exposures=False)
        self.surface = pg.Surface(size, depth=32)

    def update(self, rectangles):
        # Each request stays below the server's negotiated X11 request limit.
        maximum = self.display.display.info.max_request_length * 4 - 32
        for rect in rectangles:
            rect = self.pg.Rect(rect).clip(self.surface.get_rect())
            if not rect.width or not rect.height:
                continue
            rows = max(1, min(64, maximum // (rect.width * 4)))
            for top in range(rect.top, rect.bottom, rows):
                area = self.pg.Rect(
                    rect.left, top, rect.width, min(rows, rect.bottom - top)
                )
                pixels = self.pg.image.tobytes(
                    self.surface.subsurface(area), self.encoding
                )
                self.drawable.put_image(
                    self.gc,
                    area.x,
                    area.y,
                    area.w,
                    area.h,
                    self.X.ZPixmap,
                    self.depth,
                    0,
                    pixels,
                )
        self.display.flush()

    def close(self):
        self.gc.free()
        self.display.close()
        self.window.destroy()
