# This file contains utilities used for setting up the simulation environment.
import pybullet as p

class SimulationEnvironment:
    default_params = {
        'gravity': [0, 0, -9.8],
        'time_step': 0.001,
        'num_step': 15000,
        'camera_width': 1920,
        'camera_height': 1640,
        'rendering_options': {},
        'is_using_debug_visualizer_camera': False,
        'debug_visualizer_camera': [3, -30, -89, [0.0, 0.0, 0.0]]
    }

    def __init__(self, **kwargs):
        self.gravity = kwargs.get('gravity', self.default_params['gravity'])
        self.time_step = kwargs.get('time_step', self.default_params['time_step'])
        self.num_step = kwargs.get('num_step', self.default_params['num_step'])
        self.camera_width = kwargs.get('camera_width', self.default_params['camera_width'])
        self.camera_height = kwargs.get('camera_height', self.default_params['camera_height'])
        self.rendering_options = kwargs.get('rendering_options', self.default_params['rendering_options'])

    def GetTimeStep(self):
        return self.time_step
    
    def SetTimeStep(self, time_step):
        self.time_step = time_step
        p.setTimeStep(time_step)

    def GetNumStep(self):
        return self.num_step
    
    def SetNumStep(self, num_step):
        self.num_step = num_step

    def SetCamera(self, width=None, height=None):
        self.camera_width = width if width is not None else self.camera_width
        self.camera_height = height if height is not None else self.camera_height

    def SetRenderingOptions(self, **kwargs):
        self.rendering_options.update(kwargs)

    def _PrettyRendering(self):
        opt_str = " --width=%d --height=%d" % (self.camera_width, self.camera_height)
        for key, value in self.rendering_options.items():
            opt_str += f" --{key}={value}"
        return opt_str
    
    def _SetDebugVisualizerCamera(self, cameraDistance=None, cameraYaw=None, cameraPitch=None, cameraTargetPosition=None):
        p.resetDebugVisualizerCamera(
            cameraDistance=cameraDistance if cameraDistance is not None else self.debug_visualizer_camera[0],
            cameraYaw=cameraYaw if cameraYaw is not None else self.debug_visualizer_camera[1],
            cameraPitch=cameraPitch if cameraPitch is not None else self.debug_visualizer_camera[2],
            cameraTargetPosition=cameraTargetPosition if cameraTargetPosition is not None else self.debug_visualizer_camera[3]
        )
    
    def ExecuteSimulation(self, GUI=True):
        physicsClient = p.connect(p.GUI, options=self.PrettyRendering()) if GUI else p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(enableFileCaching=0, enableConeFriction=1)                                        
        p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE))
        p.setRealTimeSimulation(0)
        if GUI:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
        # TODO : Not Completed
        # return physicsClient
