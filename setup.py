from setuptools import setup

package_name = 'mi_robot_navegacion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebastian',
    maintainer_email='s.guayacan@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'mi_robot_interfaz = mi_robot_navegacion.mi_robot_interfaz:main',
          'mi_robot_pose = mi_robot_navegacion.mi_robot_pose:main',
          'navigation_test = mi_robot_navegacion.navigation_test:main',
          'Serial_proyecto= mi_robot_navegacion.Serial_proyecto:main',
          'mi_robot_planeacion= mi_robot_navegacion.mi_robot_planeacion:main',
          'retroalimentacion_robot= mi_robot_navegacion.retroalimentacion_robot:main',
          'mi_robot_naveacion_percepcion= mi_robot_navegacion.mi_robot_naveacion_percepcion:main',
          'mi_robot_naveacion_manipulacion= mi_robot_navegacion.mi_robot_naveacion_manipulacion:main',
        ],
    },
)
