from setuptools import setup

package_name = '{{cookiecutter.package_name}}'

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
    maintainer='{{cookiecutter.full_name}}',
    maintainer_email='{{cookiecutter.email}}',
    description='{{cookiecutter.short_description}}',
    license='MIT License @ {{cookiecutter.full_name}} {{cookiecutter.year}}',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = {{cookiecutter.package_name}}.main:main'
        ],
    },
)
