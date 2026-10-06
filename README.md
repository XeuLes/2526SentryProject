# WHURM2026_sentry

<div align=center>

<!-- ![Logo](./doc/pic/Logo6.png) -->

</div>

## 简介

> 也许不是最好的？但一定是最适合上手的C板电控框架！

本项目是基于 DJI StandardRobot++ 北极熊的通用型机器人代码框架改造的WHU哨兵机器人电控工作框架。

在原有基础上，增添了WHU新哨兵（双Yaw哨兵）与牢哨兵（官步改）的电控代码，以及SMC控制器等实用components。

详细信息见[StandardRobotpp](./README1.md)。


## 开发工具

使用VSCode作为代码编辑器，Keil5作为IDE。
> 虽然keil5老，但在嵌入式调试以及环境配置方面是非常简单易上手的。**千万不要用keil5来编辑代码！！！**
\
\
> VSCode作为现代化代码编辑器 ，通过各种插件即可实现各种提高效率的开发方式，**尤其是Github Copilot**

`.vscode` 文件夹已经提供了很多非常好用的插件，在`扩展`页面输入 `@recommended` 即可获取这些插件，点击安装即可。

[LOG.md](./doc/LOG.md) 记录了大家在开发过程中的各种奇思妙想和经验。多翻一翻或许能发现惊喜哦。大家有什么想法也都可以往里面写。

[API.md](./doc/API.md) 为开发过程中所使用的模块接口标准，若在编写代码时发现代码与标准冲突，基于本标准修改代码。

## 后续计划

具体要做的事情在[TODO](./doc/TODO.md)中

## 致谢

感谢各个战队的代码开源，所有本框架涉及到的主要的参考资料位于[reference.md](./doc/reference.md)

## 附录

本文的详细补充内容写在 [附录](./doc/appendix.md) 中。

如发现本项目中的问题，请添加至[问题列表](./doc/questions.md)并与相关负责人联系，如果有解决方案可以提交pull request。

## 通讯信息

- 作者 `Bayek` 微信：19918470930 QQ：206858817
