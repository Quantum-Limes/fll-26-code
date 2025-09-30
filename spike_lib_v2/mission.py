class Mission:
    def __init__(self, name):
        self.name = name
        self._run_func = None

    def run(self, func):
        """Decorator to set the run method of a mission instance."""
        self._run_func = func
        return func

    def execute(self, robot):
        if self._run_func:
            print(f"Running mission: {self.name}")
            return self._run_func(robot)
        else:
            print(f"No run function defined for mission: {self.name}")
            return None
