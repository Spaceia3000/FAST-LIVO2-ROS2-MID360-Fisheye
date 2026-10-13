#!/usr/bin/env python3
"""Test deployment filesystem safety in temporary directories, never production."""
import importlib.util,json,os,pathlib,tempfile,unittest
FILE=pathlib.Path(__file__).with_name('activate_imu_release.py')
spec=importlib.util.spec_from_file_location('activation',FILE)
activation=importlib.util.module_from_spec(spec);spec.loader.exec_module(activation)
class AtomicActivationTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=pathlib.Path(self.tmp.name);self.release=self.root/'releases/test';self.release.mkdir(parents=True)
        self.installed=self.root/'install/fastlivo_mapping';self.installed.parent.mkdir()
        self.old=self.root/'old_mapping';self.old.write_bytes(b'old executable bytes')
        self.new=self.release/'new_mapping';self.new.write_bytes(b'new executable bytes')
        self.installed.symlink_to(self.old)
        activation.WS=self.root;activation.INSTALLED=self.installed
        activation.command=lambda *args:'expected-sha\n'
        activation.mapped_fast=lambda:{}
        self.manifest=self.release/'release_manifest.json'
        self.manifest.write_text(json.dumps({'source_sha':'expected-sha','executable':str(self.new),
            'artifacts':{str(self.new):{'sha256':activation.sha(self.new)}}}))
    def test_switch_preserves_old_open_inode_and_rollback(self):
        inode=self.old.stat().st_ino
        with self.installed.open('rb') as old_fd:
            activation.activate(self.manifest)
            self.assertEqual(old_fd.read(),b'old executable bytes')
            self.assertEqual(self.installed.read_bytes(),b'new executable bytes')
            self.assertEqual(self.old.stat().st_ino,inode)
        activation.rollback(self.release/'activation.json')
        self.assertEqual(self.installed.resolve(),self.old)
        self.assertEqual(self.new.read_bytes(),b'new executable bytes')
    def test_refuse_changed_artifact_before_activation(self):
        self.new.write_bytes(b'changed')
        with self.assertRaisesRegex(RuntimeError,'Artifact hash changed'):
            activation.activate(self.manifest)
        self.assertEqual(self.installed.resolve(),self.old)
        self.assertFalse((self.release/'activation.json').exists())
    def test_refuse_rollback_if_user_retargeted_link(self):
        activation.activate(self.manifest)
        other=self.root/'other';other.write_bytes(b'user target')
        self.installed.unlink();self.installed.symlink_to(other)
        with self.assertRaisesRegex(RuntimeError,'Installed target changed'):
            activation.rollback(self.release/'activation.json')
        self.assertEqual(self.installed.resolve(),other)
if __name__=='__main__':unittest.main()
