#### CLONE ####
# git clone https://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@github.com/LaiYanKai/EE4308.git

#### PULL ####
# git pull origin main # git pull https://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@github.com/LaiYanKai/EE4308.git #if git remote add or git clone was used with the PAT

#### New Blank Repo created in Github, first steps ####
# git init
# git add *
# git add .gitignore
# git commit -m "First Branch"
# git branch -M main
# git remote add origin https://<PAT>@github.com/<USERNAME>/<REPONAME>.git
# git push origin main

#### Subsequent pushes ####
git add *
git add .gitignore
git commit -m "$1" # run ./git.sh "<Whatever message for commit>"
git branch -M main
git push origin main #git push https://<PAT>@github.com/<USERNAME>/<REPONAME>.git main #if git remote add or git clone not done with <PAT>
