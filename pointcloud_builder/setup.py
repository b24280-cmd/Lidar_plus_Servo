from setuptools import find_packages, setup

package_name = 'pointcloud_builder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cloud_builder  = pointcloud_builder.cloud_builder:main',
            'save_cloud     = pointcloud_builder.save_cloud:main',
            'ransac_planes  = pointcloud_builder.ransac_planes:main',
            'ransac_node    = pointcloud_builder.ransac_node:main',
            'sor_node       = pointcloud_builder.sor_node:main',
            'mls_node            = pointcloud_builder.mls_node:main',
            'pointcleannet_node  = pointcloud_builder.pointcleannet_node:main',
            'icp_node            = pointcloud_builder.icp_node:main',
        ],
    },
)
