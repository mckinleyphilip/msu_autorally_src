import argparse
import os
import sys
import random
import math

class TemplateReader(object):
    """
    Basic type intended to read .template files for world generation.
    """
    __slots__ = ['_template', '_lines']

    def __init__(self, _template):
        """
        Creates basic template reader with given filename (_template)
        """
        self._template = _template
        self._lines = ''

        #print('TemplateReader created with _template=%s'%self._template)

    def _read_template(self):
        """
        Reads template file stores its lines.
        """
        #print('TemplateReader instance trying to read with _template=%s'%self._template)
        with open(self._template) as template_file:
            self._lines = template_file.readlines() 

    def get_lines(self):
        """
        Returns lines.
        """
        return self._lines

    def encode(self):
        pass

    @staticmethod
    def decode(encoded_list, name=''):
        pass

class Obstacle(TemplateReader):
    """ 
    Child of template reader, represents obstacle to be place in a world file.
    Intedend to represent general obstacle where specific child classes will actually be
    used directly (though if pose, mass, name are enough to define an obstacle this class
    can be used directly)
    """
    __slots__ = ['_pose', '_rad_eff', '_mass', '_lines', '_name', '_type_flags'] 

    def __init__(self, _pose, **kwargs):
        """
        Creates a basic obstacle
        params:
            _pose: 6 tuple of doubles -- required
            _template: string representing template file name
            _mass: double
            _name: string
            _type_flags: flags to look for when editing templates
        """
        super(Obstacle, self).__init__(None)

        if '_rad_eff' not in kwargs.keys():
            kwargs['_rad_eff'] = 0
        if '_mass' not in kwargs.keys():
            kwargs['_mass'] = 1
        if '_name' not in kwargs.keys():
            kwargs['_name'] = 'obstacle'
        if '_type_flags' not in kwargs.keys():
            kwargs['_type_flags'] = tuple()

        self._pose = _pose
        self._mass = kwargs['_mass']
        self._name = kwargs['_name']
        self._type_flags = kwargs['_type_flags']

    def _edit_lines(self):
        """
        Edits the lines stored based on model name, pose, and mass by default.
        
        Additional lines are edited according to the self._type_flags variable
        and the corresponding _get_new_info() method.
        """

        # iterate through lines
        end_flag = False
        i = 0
        while not end_flag and i in range(len(self._lines)):
            # grab the current line and its spacing
            current_line = self._lines[i]
            spacing = current_line.split('<')[0]

            # get new info to replace that in the current line
            new_info = ''
            if 'model name=' in current_line:
                new_info = '<model name="%s">' % self._name
            elif 'pose' in current_line:
                new_info = '<pose frame="">%f %f %f %f %f %f</pose>' % self._pose
            elif 'mass' in current_line:
                new_info = '<mass>%f</mass>' % self._mass
                end_flag = len(self._type_flags) == 0 # end if no other flags to look for
            else:
                # extension used by subclasses
                for flag in self._type_flags:
                    if flag in current_line:
                        new_info = self._get_new_info(flag)
                        break

            if new_info:
                # replace line with new info and appropriate spacing
                self._lines[i] = '%s%s\n' % (spacing, new_info)
            
            i += 1

    def _get_new_info(self, flag):
        """
        Use this method to define new info for given type flags
        """
        raise NotImplementedError('Implement in child class!!!')

    def get_pose(self):
        """
        Returns pose.
        """
        return self._pose

    def get_rad_eff(self):
        """
        Returns effective radius
        """
        return self._rad_eff
    
    def build_obstacle(self):
        """ 
        Where the "magic" happens, call this to read and modify the appropriate xml template file
        to insert into a world file.
        """
        self._read_template()
        self._edit_lines()

    def encode(self):
        """
        Encodes this obstacle into a list of doubles. Add on to this in child classes to include
        more specific info.

        Right now only takes into account pose.
        """
        return list(self._pose)

    @staticmethod
    def decode(encoded_list, name=''):
        """
        Returns obstacle created from encoded list.
        """
        return Obstacle(tuple(encoded_list[:6]), _name=name)

class BoxObstacle(Obstacle):
    """
    Child of Obstacle, represents a box obstacle.
    """
    __slots__ = ['_size']

    def __init__(self, _pose, **kwargs):
        """
        Creates a box obstacle
        params:
            _pose: 6 tuple of doubles -- required
            _template: string representing template file name
            _mass: double
            _name: string
            _size: 3 tuple of doubles denoting dimension
            _type_flags: flags to look for when editing templates
        """
        if '_name' not in kwargs.keys():
            kwargs['_name'] = 'box_obstacle'
        if '_size' not in kwargs.keys():
            kwargs['_size'] = (1,1,1)
        kwargs['_type_flags'] = ('size',)

        super(BoxObstacle, self).__init__(_pose, **kwargs)

        self._template = 'box.template'
        self._size = kwargs['_size']

        # Treat it as if it is upright...
        self._rad_eff = math.sqrt(self._size[0] ** 2 + self._size[1] ** 2)/2

    def _get_new_info(self, flag):
        """
        Returns new info for the size flag.
        """
        return '<size >%f %f %f</size>' % self._size

    def encode(self):
        encoding = super(BoxObstacle, self).encode()
        encoding.append('box')

        return encoding

    @staticmethod
    def decode(encoded_list, name=''):
        return BoxObstacle(tuple(encoded_list[:6]), _name=name)

class CylinderObstacle(Obstacle):
    """
    Represents the Cylinder Obstacle.
    """
    __slots__ = ['_radius', '_length']

    def __init__(self, _pose, **kwargs):
        """
        Creates a cylinder obstacle
        params:
            _pose: 6 tuple of doubles -- required
            _template: string representing template file name
            _mass: double
            _name: string
            _radius: double
            _length: double
            _type_flags: flags to look for when editing templates
        """
        if '_name' not in kwargs.keys():
            kwargs['_name'] = 'cylinder_obstacle'
        if '_radius' not in kwargs.keys():
            kwargs['_radius'] = 0.5
        if '_length' not in kwargs.keys():
            kwargs['_length'] = 1
        kwargs['_type_flags'] = ('radius', 'length')

        super(CylinderObstacle, self).__init__(_pose, **kwargs)

        self._template = 'cylinder.template'
        self._radius = kwargs['_radius']
        self._length = kwargs['_length']

        # Treat it as if it is upright
        self._rad_eff = self._radius

    def _get_new_info(self, flag):
        """
        Returns new info for the radius and length flags.
        """
        if flag == 'radius':
            result = '<radius>%f</radius>' % self._radius
        else:
            result = '<length>%f</length>' % self._length

        return result 
    
    def encode(self):
        encoding = super(CylinderObstacle, self).encode()
        encoding.append('cylinder')

        return encoding

    @staticmethod
    def decode(encoded_list, name=''):
        return CylinderObstacle(tuple(encoded_list[:6]), _name=name)

class WorldGenerator(TemplateReader):
    """
    Represents a world generator object.
    """
    __slots__ = ['_world_name', '_obstacles', '_type_map']

    def __init__(self, _world_name, _obstacles=[]):
        """
        Creates a world generator object
        params:
            _world_name: default name to use for writing world files -- required
            _obstacles: list of obstacles to add to the next world
        """
        super(WorldGenerator, self).__init__('obstacle_sky_world.template')
        
        self._world_name = _world_name
        self._obstacles = list(_obstacles)

        self._type_map = {
                'box': {'ref': BoxObstacle((0,0,0,0,0,0)), 'constr': BoxObstacle},
                'cylinder': {'ref': CylinderObstacle((0,0,0,0,0,0)), 'constr': CylinderObstacle}
                }

    def clear(self):
        """
        Clears the list of obstacles.
        """
        self._obstacles = list()

    def detect_new_collision(self, new_obstacle):
        """
        Returns true if this object collides with any existing object in self._obstacles.
        """
        collision = False
        i = 0
        while not collision and i in range(len(self._obstacles)):
            obstacle_i = self._obstacles[i]

            d = dist(obstacle_i.get_pose()[:3], new_obstacle.get_pose()[:3])
            tol = obstacle_i.get_rad_eff() + new_obstacle.get_rad_eff()
            
            if d < tol:
                collision = True

            i += 1

        return collision


    def add_obstacle(self, obstacle):
        """
        Adds an obstacle to the list.
        """
        self._obstacles.append(obstacle)

    def add_rand_obstacles(self, num_range, obstacle_types=('box', 'cylinder')):
        """
        Adds random obstacles to the world.
        Params:
            num_range: range of how many obstacles to add
                -- accepts a single int of list [min, max]
            obstacle_types: list of obstacle types
                -- list of strings
        """
        
        # make sure the input variables are valid
        if type(num_range) is list:
            num_obstacles = random.randint(*num_range)
        elif type(num_range) is int:
            num_obstacles = num_range
        else:
            raise TypeError('Invalid type for num_range: %s' % type(num_range))
        assert all(t in ('box', 'cylinder') for t in obstacle_types)

        # approx measurements using stl file in blender
        rad_o = 800
        rad_i = 450
        ell = 1200

        # used to get convert above values:
        #   - chose 0.029 by generating obstacles and seeing if they were in the track
        #   - works pretty well for now -- not perfect
        #   - shows that above ratios are also approx correct -- not perfect
        coeff = 0.02925

        # convert above measurements
        rad_o *= coeff
        rad_i *= coeff
        ell *= coeff

        # create the appropriate number of obstacles
        for i in range(num_obstacles):
            chosen_type = random.choice(obstacle_types)
            rad_eff = self._type_map[chosen_type]['ref'].get_rad_eff()
            
            success = False
            cnt = 0
            while not success:
                pt = choose_rand_pt_on_track(rad_eff)
                pose = (pt[0], pt[1], 0.5, 0, 0, 0)
                new_obstacle = self._type_map[chosen_type]['constr'](pose,
                        _name='%s_%d'%(chosen_type, i))

                success = not self.detect_new_collision(new_obstacle)
                cnt += 1

            #if cnt > 1:
            #    print('random rechoice innefficiency: %d' % (cnt-1))
            self.add_obstacle(new_obstacle)

    def build_world(self):
        """
        Reads the template file and adds the appropriate lines for all of the obstacles.
        """
        self._read_template()

        # look for obstacle flag and grab folowing lines (from the template file lines)
        start_index = self._lines.index('        <!-- Put obstacles here -->\n')
        closing_lines = self._lines[start_index+1:]
        self._lines = self._lines[:start_index+1]

        # add lines for each obstacle
        for obstacle in self._obstacles:
            obstacle.build_obstacle()
            self._lines.extend(obstacle.get_lines())

        # add back on the closing lines
        self._lines.extend(closing_lines)

    def print_world(self, name='', suffix='', screen=False):
        """
        Prints the current world to a file with a given name (or self._world_name by default).
        Make sure to build the world first.
        """
        if not name:
            name = self._world_name
        
        name = name.split('.world')[0]
        if suffix:
            name += '-%s' % suffix
        name += '.world'

        if screen:
            for line in self._lines:
                print(line[:-1])
        else:
            with open(name, 'w') as world_file:
                for line in self._lines:
                    world_file.write(line)

    def encode(self):
        encoding = []
        for o in self._obstacles:
            encoding.append(o.encode())

        return encoding

    @staticmethod
    def decode(encoded_list, name=''):
        new_gen = WorldGenerator(name)
        for i in range(len(encoded_list)):
            o_encoding = encoded_list[i]
            if 'cylinder' in o_encoding:
                new_gen.add_obstacle(CylinderObstacle.decode(o_encoding, 'cylinder_%d'%i))
            elif 'box' in o_encoding:
                new_gen.add_obstacle(BoxObstacle.decode(o_encoding, 'box_%d'%i))
            else:
                new_gen.add_obstacle(Obstacle.decode(o_encoding, 'unknown_%d'%i))

        return new_gen

def choose_rand_pt_on_track(r_inner=13.86, r_outer=23.48, length=34.08, r_buffer=0):
    """
    Randomly choose a point on an oval track with given dimensions (uniform distribution based on
    area):
        - r_inner and r_outer describe the half circles (and track width)
        - length describes the length of the straightaway
    and the default values are based on estimates via rviz.

    Setting r_buffer to a nonzero value makes sure to account for radius of object so that it
    doesn't collide with the walls of the track (assuming the dimmensions are correct).
    """

    # get effective variable values (taking r_buffer into account)
    r_i = r_inner + r_buffer
    r_o = r_outer - r_buffer
    w = r_o - r_i
    l = length

    total_area = 2*l*w + math.pi * (r_o**2 - r_i**2)
    p_boundary = 2*l*w / total_area # used to determine whether in the straightaways or curves

    # First consider pt in a different basis (origin in center of track, x-axis parallel to
    # straightaways)

    if random.random() < p_boundary: # pt on straightaway
        # get location within straightaway rectangle 
        rectangle_pos = (random.random()*l, random.random()*w, 0)
        #print('x: %f, y: %f' % (rectangle_pos[0], rectangle_pos[1]))

        if random.random() < 0.5:
            # pt on bottom straightaway
            pt = (-l/2 + rectangle_pos[0], -r_outer + rectangle_pos[1], 0) 
        else:
            # pt on top straightaway
            pt = (-l/2 + rectangle_pos[0], r_inner + rectangle_pos[1], 0)

    else: # pt on curve
        # Note that polar coordinates stretch areas according to r -- thus to get true "trapazoidal"
        # distribution, take sqrt of num chosen uniformly between interval with squared endpoints
        r = math.sqrt(random.uniform(r_i ** 2, r_o ** 2))
        theta = random.uniform(-math.pi/2, math.pi*3/2)

        #print('\t\t\t\tr: %f, theta: %f' % (r, theta))
        # get location within annulus
        annulus_pos = (r*math.cos(theta), r*math.sin(theta), 0)

        if theta < math.pi/2:
            # pt on right curve
            pt = (l/2 + annulus_pos[0], annulus_pos[1], 0)
        else:
            # pt on left curve
            pt = (-l/2 + annulus_pos[0], annulus_pos[1], 0)

    # Now apply change of basis equivalent to the eigth rotation back to track bases
    #   -- note the rotation needed is a counter-clockwise rotation by pi/4, henc
    pt = (pt[0]-pt[1], pt[0]+pt[1], 0)
    pt = tuple(x*math.sqrt(2)/2 for x in pt)

    return pt

def dist(pt_1, pt_2, p=2):
    """
    Computes the p-normed distance between two points (use p=2 for Euclident dist)
    """
    pow_dist = 0
    for i in range(len(pt_1)):
        pow_dist += abs(pt_1[i] - pt_2[i]) ** p
    return (pow_dist) ** (1/p)

def is_number(s):
    try:
        float(s)
    except ValueError:
        return False
    return True

def main():
    """
    Method used to transform cmd line input into functionality.
    """
    parser = argparse.ArgumentParser(description='Process input to create desired worlds')
    # Encoding
    parser.add_argument('-e', dest='encode', action='store_const',
            const=True, default=False,
            help='Flag to print encoding of world instead of actual world file.')

    # Decoding
    parser.add_argument('--encoded_obj', dest='encoding', action='append', nargs='+',
            default=[],
            help='Encoding of world to generate.')
    parser.add_argument('--encoding_file', dest='encoding_file', default='', type=str,
            help='File to get encoding from -- content should be equivalent to str(encoding)')

    # Output info
    parser.add_argument('-d', dest='print_dir', type=str,
            default='~/autorally_catkin_ws/src/autorally/autorally_description/urdf',
            help='Directory of where to put the output file.')
    parser.add_argument('-n', dest='filename', type=str,
            default='generated_obstacle_sky_world.world',
            help='Name of file for result.')
    parser.add_argument('--screen', dest='use_screen', action='store_const',
            const=True, default=False,
            help='Prints output to the terminal instead of a file.')

    # World specification params
    parser.add_argument('--num_obstacles', dest='num_obstacles', type=int,
            default=10,
            help='Number of obstacles to add to the world (if no encoding is used).')

    # Prints params and their values:
    args = parser.parse_args()
    args_dict = vars(args)

    print('\n---------------------------------')
    print('Parameters:\n')
    for k in args_dict:
        print('%-20s%s' % (k, args_dict[k]))

    _ = raw_input('\nContinue? (press ENTER)')
    print('---------------------------------\n')

    # Form path name from given directory and filename
    output_path = '%s/%s' % (args.print_dir, args.filename.split('.world')[0])
    output_name = output_path.split('.world')[0]

    # Create generator
    if args.encoding or args.encoding_file:
        if args.encoding:
            encoding = []
            for raw_obstacle_enc in args.encoding:
                obstacle_enc = []
                for el in raw_obstacle_enc:
                    if is_number(el):
                        obstacle_enc.append(float(el))
                    else:
                        obstacle_enc.append(el)
                encoding.append(obstacle_enc)
        else:
            # still need to implement reading from a file...
            encoding = []

        #print('Using the following encoding:')
        #print(str(encoding))
        gen = WorldGenerator.decode(encoding, '_')
    else:
        gen = WorldGenerator('_')
        gen.add_rand_obstacles(args.num_obstacles)

    # Create and send output
    if args.encode:
        world_encoding = gen.encode()
        if args.use_screen:
            print(str(world_encoding))
        else:
            with open(output_path) as f:
                f.write(str(world_encoding))
    else:
        gen.build_world()
        gen.print_world(name=output_name, screen=args.use_screen)

if __name__ == '__main__':
    main()
