# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|
  # Ubuntu 14.04 so it can run VS Code
  config.vm.box = "ubuntu/trusty32"

  config.vm.provider "virtualbox" do |vb|
    # Display the VirtualBox GUI when booting the machine
    vb.gui = true

    # Allow symlinks
    vb.customize ["setextradata", :id, "VBoxInternal2/SharedFoldersEnableSymlinksCreate/cross-compiler", "1"]
    # Otherwise the compile will go into swap, making things slow
    vb.customize ["modifyvm", :id, "--memory", "2048"]
  end

  config.vm.provision "shell", inline: "sudo apt-get update"

  # Fix broken dictionaries-common package other stuff depends on
  config.vm.provision "shell", inline: "sudo apt-get install -y dictionaries-common"
  config.vm.provision "shell", inline: "sudo /usr/share/debconf/fix_db.pl"
  config.vm.provision "shell", inline: "sudo dpkg-reconfigure dictionaries-common "
  config.vm.provision "shell", inline: "sudo apt-get install -y dictionaries-common"

  # Install xfce desktop and virtualbox additions
  config.vm.provision "shell", inline: "sudo apt-get install -y xfce4 virtualbox-guest-dkms virtualbox-guest-utils virtualbox-guest-x11"
  # Permit anyone to start the GUI
  config.vm.provision "shell", inline: "sudo sed -i 's/allowed_users=.*$/allowed_users=anybody/' /etc/X11/Xwrapper.config"

  # install ardupilot dependencies
  config.vm.provision "shell", path: "Tools/scripts/install-prereqs-ubuntu.sh", args: ["-y", "-p"]

  # ardupilot folder from host
  config.vm.synced_folder ".", "/home/vagrant/ardupilot"

  # private IP for MissionPlanner connection
  config.vm.network "private_network", ip: "192.168.50.4"
end
