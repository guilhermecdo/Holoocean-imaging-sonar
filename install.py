import  holoocean
#oloocean.install("Ocean", branch="develop")

holoocean.packagemanager.prune()
print(holoocean.packagemanager.available_packages())
print(holoocean.packagemanager.installed_packages())