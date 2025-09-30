from .ui import Page

class Mission:
    missions = []

    @staticmethod
    def mission(name, icon=None, image=None, delta=0):
        def decorator(func):
            page = Page(func, icon=icon, image=image, delta=delta)
            Mission.missions.append(page)
            return func
        return decorator
